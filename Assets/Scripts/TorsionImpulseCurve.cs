using UnityEngine;
using System.Collections.Generic;

namespace Telescopes
{
    [RequireComponent(typeof(LineRenderer))]
    public class TorsionImpulseCurve : MonoBehaviour, IParameterizedCurve
    {
        List<CurveSegment> segments;
        LineRenderer lRenderer;

        List<Vector3> displayPoints;

        public DCurveBulb StartBulb;
        public DCurveBulb EndBulb;

        public int NumSegments { get { return segments.Count; } }

        public float ArcLength
        {
            get
            {
                float len = 0;
                foreach (CurveSegment seg in segments)
                {
                    len += seg.arcLength;
                }
                return len;
            }
        }

        public void InitFromData(List<float> impulses, List<float> arcSteps,
            float curvature, float torsion,
            OrthonormalFrame initialFrame, Vector3 initialPos)
        {
            if (initialFrame.T.magnitude < 0.01f
                || initialFrame.B.magnitude < 0.01f
                || initialFrame.N.magnitude < 0.01f) {

                throw new System.Exception("Initial frame has zeroes");
            }

            displayPoints = new List<Vector3>();
            lRenderer = GetComponent<LineRenderer>();
            segments = new List<CurveSegment>();
            // Create the initial segment
            CurveSegment prevHelix = new CurveSegment(initialPos, curvature, -torsion, 0, arcSteps[0], initialFrame);
            segments.Add(prevHelix);
            AddPointsOfSegment(prevHelix);

            float len = arcSteps[0];

            // For each impulse, make a new segment rotated by that angle.
            for (int i = 1; i < impulses.Count; i++)
            {
                float impulse = impulses[i];
                float arcStep = arcSteps[i];
                // New start point is the end of the previous curve segment.
                Vector3 newBase = TransformedHelixPoint(prevHelix, prevHelix.arcLength);
                // New frame is the frame rotated to the end of the segment.
                OrthonormalFrame newFrame = TransformedHelixFrame(prevHelix, prevHelix.arcLength);

                // Apply the torsion impulse as well.
                Quaternion impulseRot = Quaternion.AngleAxis(Mathf.Rad2Deg * impulse, newFrame.T);
                newFrame = newFrame.RotatedBy(impulseRot);

                prevHelix = new CurveSegment(newBase, curvature, -torsion, impulse, arcStep, newFrame);

                len += arcStep;

                AddPointsOfSegment(prevHelix);
                segments.Add(prevHelix);
            }

            // Add the last point of the last curve
            displayPoints.Add(TransformedHelixPoint(prevHelix, arcSteps[arcSteps.Count - 1]));

            // Set up line renderer
            lRenderer.SetVertexCount(displayPoints.Count);
            lRenderer.SetPositions(displayPoints.ToArray());
            lRenderer.SetWidth(0.1f, 0.1f);
        }

        public Vector3 StartTangent
        {
            get
            {
                return segments[0].frame.T;
            }
        }

        public Vector3 EndTangent
        {
            get
            {
                CurveSegment last = segments[segments.Count - 1];
                OrthonormalFrame endFrame = TransformedHelixFrame(last, last.arcLength);
                return endFrame.T;
            }
        }

        public Vector3 EndPosition
        {
            get
            {
                CurveSegment last = segments[segments.Count - 1];
                Vector3 endPos = TransformedHelixPoint(last, last.arcLength);
                return endPos;
            }
        }

        /// <summary>
        /// Rotate this entire curve by the given rotation.
        /// </summary>
        /// <param name="rotation"></param>
        public void Rotate(Quaternion rotation)
        {
            OrthonormalFrame initFrame = segments[0].frame.RotatedBy(rotation);
            CurveSegment initSegment = segments[0];
            initSegment.frame = initFrame;
            segments[0] = initSegment;

            CurveSegment prevHelix = segments[0];
            float len = prevHelix.arcLength;
            
            displayPoints.Clear();
            AddPointsOfSegment(prevHelix);

            for (int i = 1; i < segments.Count; i++)
            {
                // New start point is the end of the previous curve segment.
                Vector3 newBase = TransformedHelixPoint(prevHelix, prevHelix.arcLength);
                // New frame is the frame rotated to the end of the segment.
                OrthonormalFrame newFrame = TransformedHelixFrame(prevHelix, prevHelix.arcLength);

                // Apply the torsion impulse as well.
                Quaternion impulseRot = Quaternion.AngleAxis(Mathf.Rad2Deg * segments[i].impulse, newFrame.T);
                newFrame = newFrame.RotatedBy(impulseRot);

                prevHelix = new CurveSegment(newBase, segments[i].curvature, segments[i].torsion,
                    segments[i].impulse, segments[i].arcLength, newFrame);

                len += segments[i].arcLength;

                AddPointsOfSegment(prevHelix);
                segments[i] = prevHelix;
            }

            // Add the last point of the last curve
            displayPoints.Add(TransformedHelixPoint(prevHelix, prevHelix.arcLength));

            // Set up line renderer
            lRenderer.SetVertexCount(displayPoints.Count);
            lRenderer.SetPositions(displayPoints.ToArray());
            lRenderer.SetWidth(0.1f, 0.1f);
        }

        /// <summary>
        /// Translates this entire curve by the given vector.
        /// </summary>
        /// <param name="offset"></param>
        public void Translate(Vector3 offset)
        {
            for (int i = 0; i < segments.Count; i++)
            {
                CurveSegment cs = segments[i];
                cs.startPosition += offset;
                segments[i] = cs;
            }

            displayPoints.Clear();

            foreach (CurveSegment seg in segments)
            {
                AddPointsOfSegment(seg);
            }

            CurveSegment last = segments[segments.Count - 1];

            // Add the last point of the last curve
            displayPoints.Add(TransformedHelixPoint(last, last.arcLength));

            // Set up line renderer
            lRenderer.SetVertexCount(displayPoints.Count);
            lRenderer.SetPositions(displayPoints.ToArray());
            lRenderer.SetWidth(0.1f, 0.1f);
        }

        public void RotateAndOffset(Quaternion rotation, Vector3 bulbCenter, float radius)
        {
            Rotate(rotation);
            Vector3 currentStart = segments[0].startPosition;
            Vector3 desired = bulbCenter + (radius * StartTangent);
            Vector3 offset = desired - currentStart;
            Translate(offset);
        }

        public void SetMaterial(Material mat)
        {
            lRenderer.material = mat;
        }

        public void SetColor(Color c)
        {
            lRenderer.SetColors(c, c);
        }

        void AddPointsOfSegment(CurveSegment seg)
        {
            // Sampple the helix roughly every 0.1 distance
            int numSegments = Mathf.CeilToInt(seg.arcLength / 0.1f);
            float segLength = seg.arcLength / numSegments;
            // Sample the points.
            float cumulativeLength = 0;
            for (int i = 0; i < numSegments; i++)
            {
                displayPoints.Add(TransformedHelixPoint(seg, cumulativeLength));
                cumulativeLength += segLength;
            }
        }

        OrthonormalFrame TransformedHelixFrame(CurveSegment cs, float arcLen)
        {
            // Apply the local helix rotation.
            Quaternion oldRot = Quaternion.LookRotation(cs.frame.T, cs.frame.N);
            Quaternion newRot = oldRot *
                TelescopeUtils.RotateAlongHelix(cs.curvature, cs.torsion, arcLen)
                * Quaternion.Inverse(oldRot);
            OrthonormalFrame newFrame = cs.frame.RotatedBy(newRot);
            return newFrame;
        }

        Vector3 TransformedHelixPoint(CurveSegment cs, float arcLen)
        {
            // Get the helix point in local coordinates
            Vector3 helixPoint = TelescopeUtils.TranslateAlongHelix(cs.curvature, cs.torsion, arcLen);
            // Transform to world
            Quaternion rotateToWorld = Quaternion.LookRotation(cs.frame.T, cs.frame.N);
            Vector3 world = rotateToWorld * helixPoint + cs.startPosition;
            return world;
        }

        public TelescopingSegment MakeTelescope(float startRadius, bool reverse = false)
        {
            List<TelescopeParameters> tParams = new List<TelescopeParameters>();

            CurveSegment initialSeg = (reverse) ? segments[segments.Count - 1] : segments[0];

            float wallThickness = Constants.DEFAULT_WALL_THICKNESS;
            float currRadius = startRadius;

            TelescopeParameters initial = new TelescopeParameters(initialSeg.arcLength, currRadius,
                Constants.DEFAULT_WALL_THICKNESS, initialSeg.curvature, initialSeg.torsion, 0);
            tParams.Add(initial);

            for (int i = 1; i < segments.Count; i++)
            {
                int index = (reverse) ? segments.Count - 1 - i : i;

                currRadius -= wallThickness;

                float curvature = (reverse) ? segments[index].curvature : segments[index].curvature;
                float torsion = (reverse) ? segments[index].torsion : segments[index].torsion;
                float impulse = (reverse) ? -segments[index + 1].impulse : -segments[index].impulse;
                impulse *= Mathf.Rad2Deg;

                TelescopeParameters p = new TelescopeParameters(segments[index].arcLength, currRadius,
                    wallThickness, curvature, torsion, impulse);
                tParams.Add(p);
            }

            GameObject obj = new GameObject();
            obj.name = "curveApproxTelescope";

            TelescopingSegment segment = obj.AddComponent<TelescopingSegment>();
            segment.paramMode = SegmentParametersMode.Concrete;
            segment.material = DesignerController.instance.defaultTelescopeMaterial;

            if (reverse)
            {
                CurveSegment lastSeg = segments[segments.Count - 1];
                obj.transform.position = TransformedHelixPoint(lastSeg, lastSeg.arcLength);

                OrthonormalFrame initFrame = TransformedHelixFrame(lastSeg, lastSeg.arcLength);
                segment.initialDirection = -initFrame.T;
                segment.initialUp = initFrame.N;
            }
            else
            {
                obj.transform.position = segments[0].startPosition;
                segment.initialDirection = segments[0].frame.T;
                segment.initialUp = segments[0].frame.N;
            }

            segment.MakeShellsFromConcrete(tParams);

            if (reverse) segment.ReverseTelescope();

            return segment;
        }

        DiscreteCurve ToDiscreteCurve()
        {
            DiscreteCurve[] old = FindObjectsOfType<DiscreteCurve>();
            foreach (DiscreteCurve curve in old) Destroy(curve.gameObject);

            GameObject obj = new GameObject();
            obj.name = "ImpulseDCurve";
            DiscreteCurve dc = obj.AddComponent<DiscreteCurve>();

            dc.InitFromPoints(displayPoints, 0.1f);
            
            LineRenderer lr = dc.GetComponent<LineRenderer>();
            lr.SetWidth(0.1f, 0.1f);
            lr.material = lRenderer.material;
            lr.SetColors(Color.white, Color.white);

            Destroy(gameObject);
            return dc;
        }

        void Update()
        {
            if (Input.GetKey("left shift") && Input.GetKeyDown("u"))
            {
                ToDiscreteCurve();
            }
        }
    }

    struct CurveSegment
    {
        public Vector3 startPosition;
        // Constant curvature throughout this segment.
        public float curvature;
        // Torsion impulse relative to the previous segment.
        public float impulse;
        // Arc measured from the beginning of this segment.
        public float arcLength;
        // Orthonormal frame at the start of this segment.
        public OrthonormalFrame frame;
        // Constant torsion throughout this segment.
        public float torsion;

        public CurveSegment(Vector3 basePos, float curv, float tors, float imp, float len, OrthonormalFrame f)
        {
            startPosition = basePos;
            curvature = curv;
            torsion = tors;
            impulse = imp;
            arcLength = len;
            frame = f;
        }

        public void PrintSegment()
        {
            Debug.Log("Curve segment (start = " + startPosition + ", c = " + curvature +
                ", i = " + impulse + ", a = " + arcLength + ", t = " + torsion + ")");
        }
    } 
}