using UnityEngine;
using System.Collections.Generic;

using System.IO;

using NumericsLib;
using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace Telescopes
{
    [RequireComponent(typeof(LineRenderer))]
    public partial class DiscreteCurve : MonoBehaviour, IParameterizedCurve
    {
        public Vector3 StartingPoint;

        private Vector3 startingTangent;
        private Vector3 startingBinormal;

        public Vector3 StartPosition
        {
            get { return StartingPoint; }
        }

        public Vector3 LastPoint
        {
            get
            {
                return discretizedPoints[discretizedPoints.Count - 1].position;
            }
        }

        public DiscreteCurve ParentCurve;

        private Vector3 targetEndPoint;

        private List<Vector3> curvePoints;
        private List<DCurvePoint> discretizedPoints;
        private float segmentLength;

        private LineRenderer lineRender;

        private FlowMode flowMode;

        public DCurveBulb parentBulb;
        public DCurveBulb childBulb;

        public SplineCanvas containingCanvas;

        static int CurveNumber = 0;

        public float BulbShrinkRatio = 1f;
        float ActualShrinkRatio = 1f;

        public static int NextCurveNumber()
        {
            return CurveNumber++;
        }

        // Use this for initialization
        void Start()
        {
            flowMode = FlowMode.None;
        }

        // Compute all internal bending axes/angles from a list of points.
        public void InitFromPoints(List<Vector3> points, float segLength)
        {
            segmentLength = segLength;

            StartingPoint = points[0];
            startingTangent = points[1] - points[0];
            startingTangent.Normalize();

            discretizedPoints = new List<DCurvePoint>();

            Vector3 prevBinormal = Vector3.zero;

            // We need to store bending angles / directions of all the interior
            // vertices of the curve (but not the endpoints).
            for (int i = 1; i < points.Count - 1; i++)
            {
                Vector3 previousVec = points[i] - points[i - 1];
                Vector3 nextVec = points[i + 1] - points[i];

                previousVec.Normalize();
                nextVec.Normalize();

                // If zero, then treat it as a straight segment
                if (nextVec.magnitude < 0.5f)
                {
                    DCurvePoint d = new DCurvePoint(prevBinormal, 0, 0);
                    discretizedPoints.Add(d);
                    continue;
                }

                Vector3 curvatureBinormal = Vector3.Cross(previousVec, nextVec).normalized;
                if (i == 1) startingBinormal = curvatureBinormal;

                // Compute bending angles (discrete curvature).
                float dot = Vector3.Dot(previousVec, nextVec);
                float bendAngle = (dot >= 1) ? 0 : Mathf.Rad2Deg * Mathf.Acos(dot);

                // Compute twisting angles (discrete torsion).
                float twistAngle;
                // Compute twist angles as we go along.
                // The first vertex is considered to have no twist.
                if (i == 1)
                    twistAngle = 0;
                else
                {
                    twistAngle = TelescopeUtils.AngleBetween(prevBinormal, curvatureBinormal, previousVec);

                    // If the bend angle is tiny, then the curve is basically straight, so
                    // just set twist values to 0 to avoid unnecessary twisting.
                    /*if (Mathf.Abs(bendAngle) <= 0.1)
                    {
                        bendAngle = 0;
                        twistAngle = 0;
                    }*/
                }

                if (float.IsNaN(bendAngle)) throw new System.Exception("Bend angle is nan, dot = " + dot);
                if (float.IsNaN(twistAngle)) throw new System.Exception("Twist angle is nan");

                prevBinormal = curvatureBinormal;

                DCurvePoint dcp = new DCurvePoint(curvatureBinormal.normalized,
                    bendAngle, twistAngle);

                discretizedPoints.Add(dcp);
            }

            if (startingBinormal.magnitude < 0.001f)
            {
                if (startingTangent == Vector3.up)
                    startingBinormal = Vector3.right;
                else
                {
                    startingBinormal = Vector3.up;
                    Vector3 orthogonal = Vector3.Dot(startingBinormal, startingTangent) * startingTangent;
                    startingBinormal = startingBinormal - orthogonal;
                    startingBinormal.Normalize();
                }
            }

            targetEndPoint = ReconstructFromAngles();
            ComputeFrenetFrames();
            ComputeBishopFrames();
        }

        /// <summary>
        /// Apply the given rotation to this entire curve.
        /// </summary>
        /// <param name="rotation"></param>
        public void Rotate(Quaternion rotation)
        {
            startingTangent = rotation * startingTangent;
            startingBinormal = rotation * startingBinormal;
        }

        void Scale(float factor)
        {
            segmentLength *= factor;
        }

        void RealignWithParentBulb()
        {
            if (parentBulb)
            {
                Vector3 parentCenter = parentBulb.transform.position;
                Vector3 offset = parentBulb.radius * startingTangent;

                Vector3 translation = (parentCenter + offset) - StartingPoint;

                StartingPoint += translation;
                targetEndPoint += translation;
            }
        }

        /// <summary>
        /// Apply the rotation and fix the base of this curve to originate
        /// from the right point on the bulb.
        /// </summary>
        /// <param name="rotation"></param>
        /// <param name="bulbCenter"></param>
        /// <param name="radius"></param>
        public void RotateAndOffset(Quaternion rotation, Vector3 bulbCenter, Vector3 tangent, float radius)
        {
            Rotate(rotation);
            StartingPoint = bulbCenter + (radius * tangent);

            targetEndPoint = ReconstructFromAngles();
            ComputeFrenetFrames();
            ComputeBishopFrames();
        }

        public Vector3 StartTangent
        {
            get
            {
                return startingTangent;
            }
        }

        public Vector3 EndTangent
        {
            get
            {
                return discretizedPoints[discretizedPoints.Count - 1].tangent;
            }
        }

        public Vector3 EndPosition
        {
            get
            {
                return curvePoints[curvePoints.Count - 1];
            }
        }

        Vector3 ReconstructFromAngles()
        {
            curvePoints = new List<Vector3>();

            Vector3 currentPoint = StartingPoint;
            curvePoints.Add(currentPoint);
            Vector3 currentDir = startingTangent;

            currentPoint += currentDir * segmentLength;
            curvePoints.Add(currentPoint);

            Vector3 currentBinormal = startingBinormal;

            // Rotate the direction about the current binormal by the given angle,
            // and then offset to reach the next point.
            for (int i = 0; i < discretizedPoints.Count; i++)
            {
                DCurvePoint dcp = discretizedPoints[i];
                dcp.position = currentPoint;
                currentBinormal = Quaternion.AngleAxis(dcp.twistingAngle, currentDir) * currentBinormal;
                
                Quaternion nextRot = Quaternion.AngleAxis(dcp.bendingAngle, currentBinormal);
                currentDir = nextRot * currentDir;
                currentPoint += currentDir * segmentLength;

                if (float.IsNaN(currentPoint.x))
                {
                    Debug.Log("Binormal = " + currentBinormal);
                    throw new System.Exception("NaN");
                }

                curvePoints.Add(currentPoint);
            }
            SetupLineRenderer();

            return currentPoint;
        }

        void ReconstructAndAlign()
        {
            if (!Constants.PIN_ENDPOINTS)
            {
                ReconstructFromAngles();
            }
            
            else
            {
                Vector3 currentEnd = ReconstructFromAngles();

                Vector3 currentDir = (currentEnd - StartingPoint).normalized;
                Vector3 targetDir = targetEndPoint - StartingPoint;
                float targetLength = targetDir.magnitude;
                targetDir /= targetLength;

                Quaternion toTarget = Quaternion.FromToRotation(currentDir, targetDir);

                Rotate(toTarget);

                Vector3 rotatedEnd = ReconstructFromAngles();
                float currentDist = Vector3.Distance(rotatedEnd, StartingPoint);
                float scaleFactor = targetLength / currentDist;

                Scale(scaleFactor);
                RealignWithParentBulb();

                ReconstructFromAngles();
            }
        }

        void SetupLineRenderer()
        {
            lineRender = GetComponent<LineRenderer>();
            lineRender.SetVertexCount(curvePoints.Count);
            lineRender.SetPositions(curvePoints.ToArray());

            if (startingBinormal.magnitude < 0.1f)
                lineRender.SetColors(Color.blue, Color.blue);
        }

        void ComputeFrenetFrames()
        {
            if (discretizedPoints.Count <= 2) return;
            for (int i = 0; i < discretizedPoints.Count; i++)
            {
                DCurvePoint dcp = discretizedPoints[i];
                if (i == 0)
                {
                    Vector3 prevPos = StartingPoint;
                    dcp.ComputeFrenet(prevPos, discretizedPoints[i + 1]);
                }
                else if (i == discretizedPoints.Count - 1)
                {
                    Quaternion r = Quaternion.AngleAxis(discretizedPoints[i].bendingAngle,
                        discretizedPoints[i - 1].frenetFrame.B);
                    dcp.frenetFrame = discretizedPoints[i - 1].frenetFrame.RotatedBy(r);
                }
                else
                {
                    DCurvePoint prevPoint = discretizedPoints[i - 1];
                    DCurvePoint nextPoint = discretizedPoints[i + 1];
                    dcp.ComputeFrenet(prevPoint, nextPoint);
                }
            }

            FixFrenetFrames();
        }

        /// <summary>
        /// Resolve points where the Frenet frame is undefined due to
        /// the tangent being constant (i.e. zero derivative).
        /// </summary>
        void FixFrenetFrames()
        {
            // Find the index of the first valid frame.
            int firstFrameIndex = -1;
            for (int i = 0; i < discretizedPoints.Count; i++)
            {
                OrthonormalFrame frame = discretizedPoints[i].frenetFrame;
                if (frame.B.magnitude > 0)
                {
                    firstFrameIndex = i;
                    break;
                }
            }

            // If there were no valid Frenet frames, then the entire curve is just
            // a striaght line. We just assign an arbitrary
            // frame aligned with the tangent for each one.
            if (firstFrameIndex == -1)
            {
                Vector3 tangent = discretizedPoints[0].frenetFrame.T;
                Vector3 normal, binormal;
                // If the tangent is pointing up, use an orthogonal direction
                if (tangent == Vector3.up)
                {
                    binormal = Vector3.right;
                    normal = Vector3.Cross(binormal, tangent);
                }
                // Otherwise take some vector orthogonal to the tangent,
                // by projecting the up direction onto it
                else
                {
                    binormal = Vector3.up;
                    binormal = binormal - Vector3.Dot(binormal, tangent) * tangent;
                    binormal.Normalize();
                    normal = Vector3.Cross(binormal, tangent);
                }

                startingBinormal = binormal;
                OrthonormalFrame defaultFrame = new OrthonormalFrame(tangent, normal, binormal);

                foreach (DCurvePoint dcp in discretizedPoints)
                {
                    dcp.frenetFrame = defaultFrame;
                }
            }

            // Otherwise there is at least one valid frame, so propagate to all missing frames.
            else
            {
                for (int i = 0; i < discretizedPoints.Count; i++)
                {
                    // If the current frame is invalid (has zero binormal), fix it
                    if (discretizedPoints[i].frenetFrame.B.magnitude == 0)
                    {
                        if (i < firstFrameIndex) FixFrenetForward(i);
                        else if (i > firstFrameIndex) FixFrenetBackward(i);
                    }
                }
            }
        }

        void FixFrenetForward(int start)
        {
            int validIndex = start + 1;
            // Search forward until we find a valid Frenet frame
            while (discretizedPoints[validIndex].frenetFrame.B.magnitude == 0)
            {
                validIndex++;
            }

            // Un-rotate the valid frame so that it's aligned with the current tangent
            OrthonormalFrame nextFrame = discretizedPoints[validIndex].frenetFrame;
            Vector3 nextTangent = nextFrame.T;
            Vector3 currentTangent = discretizedPoints[validIndex - 1].frenetFrame.T;
            Quaternion reverseRotation = Quaternion.FromToRotation(nextTangent, currentTangent);
            OrthonormalFrame backFrame = nextFrame.RotatedBy(reverseRotation);

            // Set the aligned frame to all of the points missing frames
            for (int i = start; i < validIndex; i++)
            {
                discretizedPoints[i].frenetFrame = backFrame;
            }
        }

        void FixFrenetBackward(int start)
        {
            // Find the most recent valid frame
            int validIndex = start - 1;
            while (discretizedPoints[validIndex].frenetFrame.B.magnitude == 0)
            {
                validIndex--;
            }

            OrthonormalFrame validFrame = discretizedPoints[validIndex].frenetFrame;

            // No need to un-rotate because the tangents are already aligned.
            for (int i = start; i > validIndex; i--)
            {
                discretizedPoints[i].frenetFrame = validFrame;
            }
        }

        void ComputeBishopFrames()
        {
            if (discretizedPoints.Count <= 2) return;
            for (int i = 0; i < discretizedPoints.Count; i++)
            {
                DCurvePoint dcp = discretizedPoints[i];
                if (i == 0)
                {
                    dcp.bishopFrame = dcp.frenetFrame;
                }
                else if (i == discretizedPoints.Count - 1)
                {
                    dcp.bishopFrame = discretizedPoints[i - 1].bishopFrame;
                }
                else
                {
                    DCurvePoint prevPoint = discretizedPoints[i - 1];
                    DCurvePoint nextPoint = discretizedPoints[i + 1];
                    dcp.PropagateBishop(prevPoint, nextPoint, prevPoint.bishopFrame);
                }
            }
        }

        // Update is called once per frame
        void Update()
        {
            if (Input.GetKey("left shift") && Input.GetKeyDown("j"))
            {
                List<float> points = EvenlySpacedPoints(ComputeNumImpulses());
                SolveImpulsesVariableQP(points);
                
                //Debug.Log("Segmenting curvature into " + DesignerController.instance.NumKMeansPoints + " segments");
                //CurvatureKMeans(DesignerController.instance.NumKMeansPoints);
            }

            if (Input.GetKey("left shift") && Input.GetKeyDown("l"))
            {
                bool isCtrlKeyDown = Input.GetKey(KeyCode.LeftControl) || Input.GetKey(KeyCode.RightControl);
                if (isCtrlKeyDown)
                {
                    CurvatureToAverage();
                    ComputeFrenetFrames();
                    ComputeBishopFrames();
                }
                else
                {
                    if (flowMode != FlowMode.CurvatureFlow) flowMode = FlowMode.CurvatureFlow;
                    else flowMode = FlowMode.None;
                }
            }

            else if (Input.GetKey("left shift") && Input.GetKeyDown("k"))
            {
                bool isShiftKeyDown = Input.GetKey(KeyCode.LeftControl) || Input.GetKey(KeyCode.RightControl);
                if (isShiftKeyDown)
                {
                    TorsionToAverage();
                    ComputeFrenetFrames();
                    ComputeBishopFrames();
                }
                else
                {
                    if (flowMode != FlowMode.TorsionFlow) flowMode = FlowMode.TorsionFlow;
                    else flowMode = FlowMode.None;
                }
            }

            else if (Input.GetKey("left shift") && Input.GetKeyDown("home"))
            {
                // Dump cumulative torsion function
                float cumulativeT = 0;
                float currentArc = 0;

                List<float> cumulativeTs = new List<float>();
                List<float> arcPos = new List<float>();

                cumulativeTs.Add(cumulativeT);
                arcPos.Add(currentArc);

                foreach (DCurvePoint dcp in discretizedPoints)
                {
                    cumulativeT += dcp.twistingAngle;
                    currentArc += segmentLength;

                    cumulativeTs.Add(cumulativeT);
                    arcPos.Add(currentArc);
                }

                using (StreamWriter file = new StreamWriter("discreteCurveFn.csv"))
                {
                    for (int i = 0; i < cumulativeTs.Count; i++)
                    {
                        file.WriteLine(arcPos[i] + "," + cumulativeTs[i]);
                    }
                }
                Debug.Log("Wrote discrete curve's cumulative torsion to discreteCurveFn.csv");
            }

            if (flowMode == FlowMode.CurvatureFlow)
            {
                //CurvaturePositionFlow(0.0002f);
                
                CurvatureFlow(1f);
                ComputeFrenetFrames();
                ComputeBishopFrames();
            }

            else if (flowMode == FlowMode.TorsionFlow)
            {
                TorsionFlow(1f);
                ComputeFrenetFrames();
                ComputeBishopFrames();
            }
        }

        int ComputeNumImpulses()
        {
            int numImpulses = DesignerController.instance.numImpulses;
            int maxNumShells = Mathf.FloorToInt(ArcLength / Constants.MIN_SHELL_LENGTH);

            if (parentBulb && childBulb)
            {
                float oldLarger = Mathf.Max(parentBulb.radius, childBulb.radius);
                float smaller = Mathf.Min(parentBulb.radius, childBulb.radius);
                float larger = Mathf.LerpUnclamped(smaller, oldLarger, BulbShrinkRatio);
                ActualShrinkRatio = larger / oldLarger;
                // Compute the amount by which we have to shrink
                float radiusDiff = (larger - smaller) - ArcLength * Constants.TAPER_SLOPE;
                // Compute how much we will shrink by following the arc length of this curve
                numImpulses = Mathf.CeilToInt(radiusDiff / Constants.WALL_THICKNESS);
                numImpulses = Mathf.Max(Mathf.Min(numImpulses, maxNumShells), 2);
            }

            else if (parentBulb)
            {
                float oldLarger = parentBulb.radius;
                float smaller = Constants.DEFAULT_MIN_RADIUS;
                float larger = Mathf.LerpUnclamped(smaller, oldLarger, BulbShrinkRatio);
                ActualShrinkRatio = larger / oldLarger;

                float radiusDiff = (larger - smaller - ArcLength * Constants.TAPER_SLOPE);
                numImpulses = Mathf.CeilToInt(radiusDiff / Constants.WALL_THICKNESS);
                numImpulses = Mathf.Max(Mathf.Min(numImpulses, maxNumShells), 2);
            }

            else if (childBulb)
            {
                float oldLarger = childBulb.radius;
                float smaller = Constants.DEFAULT_MIN_RADIUS;
                float larger = Mathf.LerpUnclamped(smaller, oldLarger, BulbShrinkRatio);
                ActualShrinkRatio = larger / oldLarger;

                float radiusDiff = (larger - smaller - ArcLength * Constants.TAPER_SLOPE);
                numImpulses = Mathf.CeilToInt(radiusDiff / Constants.WALL_THICKNESS);
                numImpulses = Mathf.Max(Mathf.Min(numImpulses, maxNumShells), 2);
            }

            Debug.Log("Max num shells = " + maxNumShells + ", numImpulses = " + numImpulses);

            return numImpulses;
        }

        public TorsionImpulseCurve MakeImpulseCurve()
        {
            bool isShiftKeyDown = Input.GetKey(KeyCode.LeftControl) || Input.GetKey(KeyCode.RightControl);
            bool isAltKeyDown = Input.GetKey(KeyCode.LeftAlt) || Input.GetKey(KeyCode.RightAlt);

            int numImpulses = ComputeNumImpulses();

            List<float> points = EvenlySpacedPoints(numImpulses);
            if (isShiftKeyDown)
            {
                if (DesignerController.instance.UseSparseSolve) return SolveImpulseLPSparse(points);
                else
                {
                    var tup = SolveImpulseLP(points, makeCurve: false);
                    float slope = tup.Item1;
                    List<float> impulses = tup.Item2;
                    return MakeTorsionImpulseCurve(impulses, points, slope);
                }
            }
            else if (isAltKeyDown)
            {
                Debug.Log("Alt down");
                return SolveImpulsesVariableQP(points);
            }
            else return SolveImpulsesQP(points);
        }

        List<float> EvenlySpacedPoints(int num)
        {
            float step = ArcLength / num;
            List<float> steps = new List<float>();
            
            for (int i = 0; i < num; i++)
            {
                steps.Add(i * step);
            }
            steps.Add(ArcLength);


            return steps;
        }

        public float ArcLength
        {
            get { return (discretizedPoints.Count + 1) * segmentLength; }
        }

        void ScaleArcParameter(out int segNum, out float scaledT, float arcT)
        {
            scaledT = arcT / segmentLength;
            segNum = Mathf.FloorToInt(scaledT);
            if (segNum < 0)
            {
                segNum = 0;
                scaledT = 0;
            }
            else if (segNum >= discretizedPoints.Count)
            {
                segNum = discretizedPoints.Count - 1;
                scaledT = 0;
            }
            else
            {
                scaledT = scaledT - segNum;
            }
        }

        public Vector3 PositionAtPoint(float t)
        {
            int segNum;
            float scaledT;
            ScaleArcParameter(out segNum, out scaledT, t);

            if (segNum == discretizedPoints.Count - 1)
            {
                return discretizedPoints[segNum].position;
            }

            Vector3 pos1 = discretizedPoints[segNum].position;
            Vector3 pos2 = discretizedPoints[segNum + 1].position;

            return Vector3.Lerp(pos1, pos2, scaledT);
        }

        public OrthonormalFrame FrenetAtPoint(float t)
        {
            int segNum;
            float scaledT;
            ScaleArcParameter(out segNum, out scaledT, t);

            if (segNum == discretizedPoints.Count - 1)
            {
                return discretizedPoints[segNum].frenetFrame;
            }

            return OrthonormalFrame.Slerp(discretizedPoints[segNum].frenetFrame,
                discretizedPoints[segNum + 1].frenetFrame,
                scaledT);
        }

        public OrthonormalFrame BishopAtPoint(float t)
        {
            int segNum;
            float scaledT;
            ScaleArcParameter(out segNum, out scaledT, t);

            if (segNum == discretizedPoints.Count - 1)
            {
                return discretizedPoints[segNum].bishopFrame;
            }

            return OrthonormalFrame.Slerp(discretizedPoints[segNum].bishopFrame,
                discretizedPoints[segNum + 1].bishopFrame,
                scaledT);
        }

        float AverageCurvature()
        {
            // Get the average curvature of this curve
            float averageCurvature = 0;
            foreach (DCurvePoint dcp in discretizedPoints)
            {
                averageCurvature += dcp.bendingAngle * Mathf.Deg2Rad;
            }
            averageCurvature /= discretizedPoints.Count;
            averageCurvature = TelescopeUtils.CurvatureOfDiscrete(averageCurvature, segmentLength);
            return averageCurvature;
        }

        TorsionImpulseCurve MakeTorsionImpulseCurve(List<float> impulses, List<float> arcPoints, float constTorsion)
        {
            float averageCurvature = AverageCurvature();

            Vector3 startingNormal = Vector3.Cross(startingBinormal, startingTangent);
            OrthonormalFrame startFrame = new OrthonormalFrame(startingTangent, startingNormal, startingBinormal);
            
            List<float> arcSteps = new List<float>();
            for (int i = 0; i < arcPoints.Count - 1; i++)
            {
                arcSteps.Add(arcPoints[i + 1] - arcPoints[i]);
            }

            GameObject obj = new GameObject();
            obj.name = "TorsionApproxCurve";
            TorsionImpulseCurve impulseCurve = obj.AddComponent<TorsionImpulseCurve>();

            impulseCurve.InitFromData(impulses, arcSteps,
                averageCurvature, constTorsion,
                startFrame, StartingPoint);

            impulseCurve.SetMaterial(lineRender.material);
            impulseCurve.BulbShrinkRatio = ActualShrinkRatio;

            return impulseCurve;
        }

        TorsionImpulseCurve MakeTorsionImpulseCurve(List<float> impulses, List<float> arcPoints,
            List<float> curvatureList, List<float> torsionList)
        {
            Debug.Log("Arc length of DC = " + ArcLength);

            Vector3 startingNormal = Vector3.Cross(startingBinormal, startingTangent);
            OrthonormalFrame startFrame = new OrthonormalFrame(startingTangent, startingNormal, startingBinormal);

            List<float> arcSteps = new List<float>();
            for (int i = 0; i < arcPoints.Count - 1; i++)
            {
                arcSteps.Add(arcPoints[i + 1] - arcPoints[i]);
            }

            GameObject obj = new GameObject();
            obj.name = "TorsionApproxCurve";
            TorsionImpulseCurve impulseCurve = obj.AddComponent<TorsionImpulseCurve>();

            impulseCurve.InitFromData(impulses, arcSteps,
                curvatureList, torsionList,
                startFrame, StartingPoint);

            impulseCurve.SetMaterial(lineRender.material);

            return impulseCurve;
        }

    }

    public enum FlowMode
    {
        None, CurvatureFlow, TorsionFlow
    }

    class DCurvePoint
    {
        /// <summary>
        /// Discrete curvature is defined as the rotation angle about the binormal.
        /// </summary>
        public Vector3 binormal;
        public float bendingAngle;

        public float twistingAngle;

        public float cumulativeTwist;

        public Vector3 position;
        public Vector3 currentGradient;

        public Vector3 tangent
        {
            get { return frenetFrame.T; }
        }

        public OrthonormalFrame frenetFrame;
        public OrthonormalFrame bishopFrame;

        public DCurvePoint(Vector3 dir, float bendAngle, float twistAngle)
        {
            binormal = dir;
            bendingAngle = bendAngle;
            twistingAngle = twistAngle;
        }

        public void ComputeFrenet(Vector3 prevPos, Vector3 nextPos)
        {
            Vector3 tangent = (nextPos - position).normalized;
            Vector3 prevTangent = (position - prevPos).normalized;
            Vector3 binormal = Vector3.Cross(prevTangent, tangent).normalized;

            Vector3 normal = Vector3.Cross(binormal, tangent);

            frenetFrame = new OrthonormalFrame(tangent, normal, binormal);
        }

        public void ComputeFrenet(Vector3 prevPos, DCurvePoint next)
        {
            ComputeFrenet(prevPos, next.position);
        }

        public void ComputeFrenet(DCurvePoint prev, DCurvePoint next)
        {
            ComputeFrenet(prev.position, next.position);
        }

        public OrthonormalFrame PropagateBishop(Vector3 prevPos, Vector3 nextPos, OrthonormalFrame prevFrame)
        {
            Vector3 tangent = (nextPos - position).normalized;
            Vector3 prevTangent = (position - prevPos).normalized;
            Vector3 binormal = Vector3.Cross(prevTangent, tangent).normalized;

            float angle = TelescopeUtils.AngleBetween(prevTangent, tangent, binormal);

            Quaternion rotation = Quaternion.AngleAxis(angle, binormal);

            OrthonormalFrame rotated = prevFrame.RotatedBy(rotation);
            bishopFrame = rotated;
            return rotated;
        }

        public OrthonormalFrame PropagateBishop(DCurvePoint prev, DCurvePoint next, OrthonormalFrame prevFrame)
        {
            return PropagateBishop(prev.position, next.position, prevFrame);
        }

        public static float ToTwistAngle(DCurvePoint dcp)
        {
            return dcp.twistingAngle;
        }

        public static float ToBendAngle(DCurvePoint dcp)
        {
            return dcp.bendingAngle;
        }
    }

    public struct OrthonormalFrame
    {
        public Vector3 T;
        public Vector3 N;
        public Vector3 B;

        public OrthonormalFrame(Vector3 t, Vector3 n, Vector3 b)
        {
            T = t;
            N = n;
            B = b;
        }

        public OrthonormalFrame RotatedBy(Quaternion rotation)
        {
            Vector3 rotT = rotation * T;
            Vector3 rotN = rotation * N;
            Vector3 rotB = rotation * B;
            return new OrthonormalFrame(rotT, rotN, rotB);
        }

        public static OrthonormalFrame Slerp(OrthonormalFrame frame1, OrthonormalFrame frame2, float t)
        {
            Vector3 tangent1 = frame1.T;
            Vector3 tangent2 = frame2.T;
            Vector3 tangent = Vector3.Slerp(tangent1, tangent2, t);

            Vector3 normal1 = frame1.N;
            Vector3 normal2 = frame2.N;
            Vector3 normal = Vector3.Slerp(normal1, normal2, t);

            Vector3 binormal1 = frame1.B;
            Vector3 binormal2 = frame2.B;
            Vector3 binormal = Vector3.Slerp(binormal1, binormal2, t);

            return new OrthonormalFrame(tangent, normal, binormal);
        }
    } 
}