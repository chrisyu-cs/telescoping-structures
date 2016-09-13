using UnityEngine;
using System.Collections.Generic;

namespace Telescopes
{
    [RequireComponent(typeof(LineRenderer))]
    public class CatmullRomSpline : MonoBehaviour
    {
        public CatmullRomSpline parent;
        public List<CatmullRomSpline> children;

        public List<Vector3> points;
        private List<CatmullRomSegment> segments;
        private LineRenderer lineRender;

        public int pointsPerSegment = 20;

        private bool needsUpdate = false;
        public bool randomize = false;

        private List<DraggablePoint> spheres;

        public SplineCanvas containingCanvas;

        public DraggablePoint StartBulb;
        public DraggablePoint EndBulb;

        static int numSplinesCreated = 0;

        public int NumSegments { get { return segments.Count; } }

        void Awake()
        {
            children = new List<CatmullRomSpline>();
            lineRender = GetComponent<LineRenderer>();

            lineRender.SetWidth(0.1f, 0.1f);
        }

        // Use this for initialization
        void Start()
        {
            segments = new List<CatmullRomSegment>();
            spheres = new List<DraggablePoint>();

            if (points.Count < 4)
            {
                createSpheres();
                return;
            }

            if (randomize)
            {
                for (int i = 0; i < points.Count; i++)
                {
                    points[i] = 2f * Random.insideUnitSphere;
                    points[i] += 2.5f * Vector3.up;
                }
            }

            SetMaterial(DesignerController.instance.defaultLineMaterial);
            lineRender.SetColors(Color.white, Color.red);

            makeSegments();
            createSpheres();
            MoveEndpoints();
            updateRenderPoints();
        }

        public void DeletePoint(int index)
        {
            if (points.Count <= 1) return;

            if (points.Count <= 4)
            {
                Destroy(gameObject);
                containingCanvas.DeleteSpline(this);
            }

            points.RemoveAt(index);

            makeSegments();
            createSpheres();
            MoveEndpoints();
            updateRenderPoints();
        }

        public void DuplicatePoint(int index)
        {
            Vector3 pointToDuplicate = points[index];
            // Add a slight offset to differentiate the two
            pointToDuplicate.y += 0.1f;
            points.Insert(index, pointToDuplicate);

            makeSegments();
            updateRenderPoints();
            createSpheres();
        }

        public void SetMaterial(Material m)
        {
            lineRender.material = m;
        }

        void makeSegments()
        {
            segments.Clear();
            for (int i = 0; i < points.Count - 3; i++)
            {
                Vector3 p0 = points[i];
                Vector3 p1 = points[i + 1];
                Vector3 p2 = points[i + 2];
                Vector3 p3 = points[i + 3];

                segments.Add(new CatmullRomSegment(p0, p1, p2, p3));
            }
        }

        public static CatmullRomSpline SplineOfPoints(List<Vector3> pts)
        {
            GameObject splineObj = new GameObject();
            
            CatmullRomSpline spline = splineObj.AddComponent<CatmullRomSpline>();
            spline.points = new List<Vector3>();
            spline.points.AddRange(pts);
            spline.SetMaterial(DesignerController.instance.defaultLineMaterial);

            spline.name = "spline" + numSplinesCreated;
            numSplinesCreated++;

            return spline;
        }

        public void ReplaceWithBulb(int index)
        {
            // Do nothing if this is an endpoint -- we can only insert
            // a bulb at an interior point.
            if (index <= 1) return;
            if (index >= points.Count - 2) return;

            List<Vector3> beforePoints = new List<Vector3>();
            List<Vector3> afterPoints = new List<Vector3>();

            // We want to copy all of the points up to the split position,
            // and make a new spline out of that.
            for (int i = 0; i < index; i++)
            {
                beforePoints.Add(points[i]);
            }

            // Add the split position as the last point
            beforePoints.Add(points[index]);
            // Add the last tangent handle as a point farther down the spline
            Vector3 middleBefore = sample(index + 0.5f);
            beforePoints.Add(middleBefore);

            // We also want to make a new spline of all the points 
            // starting after the split position.

            // Add first tangent handle as coming before the split
            Vector3 middleAfter = sample(index - 0.5f);
            afterPoints.Add(middleAfter);
            // Add the split position as the first point
            afterPoints.Add(points[index]);

            for (int i = index + 1; i < points.Count; i++)
            {
                afterPoints.Add(points[i]);
            }

            // Make the splines
            CatmullRomSpline before = SplineOfPoints(beforePoints);
            containingCanvas.AddExistingSpline(before);
            
            CatmullRomSpline after = SplineOfPoints(afterPoints);
            containingCanvas.AddExistingSpline(after);
            
            // Connect splines to the bulb we just added
            DraggablePoint bulb = containingCanvas.AddBulb(points[index]);
            before.StartBulb = StartBulb;
            before.EndBulb = bulb;

            after.StartBulb = bulb;
            after.EndBulb = EndBulb;

            bulb.SetSize(0.1f);

            // Delete the current spline that was split
            containingCanvas.DeleteSpline(this);
            Destroy(gameObject);
        }

        void FixSegmentsAroundIndex(int index)
        {
            int lowerBound = Mathf.Max(0, index - 3);
            int upperBound = Mathf.Min(index, points.Count - 4);

            for (int i = lowerBound; i <= upperBound; i++)
            {
                Vector3 p0 = points[i];
                Vector3 p1 = points[i + 1];
                Vector3 p2 = points[i + 2];
                Vector3 p3 = points[i + 3];
                segments[i].ResetPositions(p0, p1, p2, p3);
            }
        }

        void MoveEndpoints()
        {
            if (points.Count < 4) return;
            if (points.Count == 4)
            {
                int l = points.Count - 1;
                Vector3 offsetBegin = (points[1] - points[2]).normalized * 0.1f;
                Vector3 offsetEnd = -offsetBegin;

                if (!StartBulb) points[0] = points[1] + offsetBegin;
                if (!EndBulb) points[l] = points[l - 1] + offsetEnd;
            }
            else
            {
                if (!StartBulb)
                {
                    Vector3 circumBegin = TelescopeUtils.Circumcenter(points[1], points[2], points[3]);
                    Vector3 normalBegin = Vector3.Cross(points[2] - points[1], points[3] - points[1]);
                    Vector3 beginOffset = points[1] - circumBegin;
                    Vector3 rotatedBegin = Quaternion.AngleAxis(-5, normalBegin) * beginOffset + circumBegin;
                    Vector3 diffBegin = (rotatedBegin - points[1]).normalized * 0.1f;
                    points[0] = points[1] + diffBegin;
                }
                
                if (!EndBulb)
                {
                    int last = points.Count - 1;
                    Vector3 circumEnd = TelescopeUtils.Circumcenter(points[last - 1], points[last - 2], points[last - 3]);
                    Vector3 normalEnd = Vector3.Cross(points[last - 2] - points[last - 1], points[last - 3] - points[last - 1]);
                    Vector3 endOffset = points[last - 1] - circumEnd;
                    Vector3 rotatedEnd = Quaternion.AngleAxis(-5, normalEnd) * endOffset + circumEnd;
                    Vector3 diffEnd = (rotatedEnd - points[last - 1]).normalized * 0.1f;
                    points[last] = points[last - 1] + diffEnd;
                }
            }

            FixSegmentsAroundIndex(0);
            spheres[0].transform.position = points[0];

            FixSegmentsAroundIndex(points.Count - 1);
            spheres[points.Count - 1].transform.position = points[points.Count - 1];
        }

        public void UpdatePosition(int index, Vector3 newPosition)
        {
            Vector3 offset = newPosition - points[index];

            points[index] = newPosition;
            MoveEndpoints();
            FixSegmentsAroundIndex(index);

            updateRenderPoints();
        }

        void updateRenderPoints()
        {
            if (segments.Count <= 0) return;
            List<Vector3> renderPoints = new List<Vector3>();

            for (int i = 0; i < segments.Count; i++)
            {
                segments[i].addSamplePointsTo(renderPoints, pointsPerSegment);
            }
            segments[segments.Count - 1].addLastPointTo(renderPoints);

            lineRender.SetVertexCount(renderPoints.Count);
            lineRender.SetPositions(renderPoints.ToArray());

            needsUpdate = false;
        }

        public DraggablePoint GetSphere(int i)
        {
            return spheres[i];
        }

        void createSpheres()
        {
            foreach (DraggablePoint p in spheres)
            {
                Destroy(p.gameObject);
            }
            spheres.Clear();

            for (int i = 0; i < points.Count; i++)
            {
                GameObject obj = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                obj.name = "splineMarker" + i;
                obj.transform.localScale = new Vector3(0.1f, 0.1f, 0.1f);
                obj.transform.position = points[i];
                obj.transform.parent = transform;

                obj.AddComponent<SphereCollider>();
                DraggablePoint draggable = obj.AddComponent<DraggablePoint>();
                draggable.index = i;
                draggable.parentSpline = this;
                draggable.containingCanvas = containingCanvas;
                draggable.Type = PointType.Spline;

                spheres.Add(draggable);
            }

            if (spheres.Count >= 4)
            {
                spheres[0].gameObject.SetActive(false);
                spheres[spheres.Count - 1].gameObject.SetActive(false);
            }
        }

        public Vector3 sample(float t)
        {
            int segNum = Mathf.FloorToInt(t);
            if (segNum >= segments.Count)
            {
                return segments[segments.Count - 1].sample(1);
            }
            float segT = t - segNum;
            return segments[segNum].sample(segT);
        }

        public Vector3 StartTangent()
        {
            return sampleTangent(0);
        }

        public Vector3 EndTangent()
        {
            return sampleTangent(NumSegments);
        }

        public Vector3 sampleTangent(float t)
        {
            int segNum = Mathf.FloorToInt(t);
            if (segNum >= segments.Count)
            {
                return segments[segments.Count - 1].sampleTangent(1).normalized;
            }
            float segT = t - segNum;
            return segments[segNum].sampleTangent(segT).normalized;
        }

        float searchNextPoint(float startT, float segmentLength)
        {
            float maxT = segments.Count;
            float diff = sampleTangent(startT).magnitude;
            Vector3 curPt = sample(startT);
            Vector3 nextPt = sample(startT + diff);
            float lowerBound = diff;
            float upperBound = diff;

            float distance = Vector3.Distance(nextPt, curPt);

            int numIterations = 0;
            int maxIterations = 100;

            // If the initial guess is already too far away, we have an upper
            // bound and need to find a lower bound. Iteratively halve the distance.
            if (distance > segmentLength)
            {
                while (distance > segmentLength && numIterations < maxIterations)
                {
                    numIterations++;
                    upperBound = diff;
                    diff /= 2;
                    nextPt = sample(startT + diff);
                    distance = Vector3.Distance(nextPt, curPt);
                }
                lowerBound = diff;
            }

            // Otherwise if the initial guess is too close, we have a lower
            // bound and need to find an upper bound. Iteratively double the distance.
            else if (distance < segmentLength)
            {
                while (distance < segmentLength && numIterations < maxIterations)
                {
                    numIterations++;
                    lowerBound = diff;
                    diff *= 2;
                    nextPt = sample(startT + diff);
                    distance = Vector3.Distance(nextPt, curPt);

                    if (startT + diff > maxT)
                    {
                        return maxT;
                    }
                }
            }

            // If the initial guess is exactly right, just return it.
            else return startT + diff;

            float midT = (lowerBound + upperBound) / 2f;
            // Now binary search to find the exact point.
            Vector3 midpoint = sample(startT + midT);
            distance = Vector3.Distance(midpoint, curPt);

            while (Mathf.Abs(segmentLength - distance) > 1e-5 && numIterations < maxIterations)
            {
                numIterations++;
                if (distance > segmentLength) upperBound = midT;
                else lowerBound = midT;
                midT = (lowerBound + upperBound) / 2f;
                midpoint = sample(startT + midT);
                distance = Vector3.Distance(midpoint, curPt);
            }

            if (numIterations >= maxIterations)
            {
                Debug.Log("Iteration cutoff reached (distance = " + distance + ")");
                return startT + midT;
            }

            return startT + midT;
        }

        List<Vector3> discretizeSpline(float segmentLength)
        {
            float maxT = segments.Count;
            float curT = 0;
            float nextT = 0;

            List<Vector3> allPoints = new List<Vector3>();

            allPoints.Add(sample(0));

            while (curT < maxT)
            {
                nextT = searchNextPoint(curT, segmentLength);
                Vector3 nextPoint = sample(nextT);
                allPoints.Add(nextPoint);
                curT = nextT;
            }

            return allPoints;
        }

        public DiscreteCurve ConvertToDCurve()
        {
            float segLength = 0.1f;

            List<Vector3> discretePoints = discretizeSpline(segLength);

            lineRender.SetVertexCount(discretePoints.Count);
            lineRender.SetPositions(discretePoints.ToArray());

            GameObject discretized = new GameObject();
            discretized.transform.parent = transform.parent;

            discretized.name = "DiscretizedCurve";
            DiscreteCurve dCurve = discretized.AddComponent<DiscreteCurve>();
            dCurve.InitFromPoints(discretePoints, segLength);

            LineRenderer lr = dCurve.GetComponent<LineRenderer>();
            lr.SetWidth(0.1f, 0.1f);
            lr.material = lineRender.material;

            gameObject.SetActive(false);

            return dCurve;
        }

        // Update is called once per frame
        void Update()
        {
            if (Input.GetKeyDown("home"))
            {
                foreach (Vector3 v in points)
                {
                    Debug.Log(v);
                }
            }

            if (Input.GetKey("left shift") && Input.GetKeyDown("p") && !parent)
            {
                ConvertToDCurve();
            }

            if (points.Count >= 4)
            {
                if (StartBulb)
                {
                    spheres[1].FollowBulb(StartBulb, StartTangent());
                    spheres[0].Move(StartBulb.transform.position);
                }
                if (EndBulb)
                {
                    spheres[spheres.Count - 2].FollowBulb(EndBulb, -EndTangent());
                    spheres[spheres.Count - 1].Move(EndBulb.transform.position);
                }
            }
        }
    }

    public class CatmullRomSegment
    {
        public Vector3 P0, P1, P2, P3;
        private float t0, t1, t2, t3;

        public CatmullRomSegment(Vector3 v0, Vector3 v1, Vector3 v2, Vector3 v3)
        {
            ResetPositions(v0, v1, v2, v3);
        }

        public void ResetPositions(Vector3 v0, Vector3 v1, Vector3 v2, Vector3 v3)
        {
            // Set points
            P0 = v0;
            P1 = v1;
            P2 = v2;
            P3 = v3;
            // Recompute tangents and stuff
            recomputeParameters();
        }

        void recomputeParameters()
        {
            // Compute parameters t0 through t3
            t0 = 0;
            t1 = computeT_j(t0, P0, P1);
            t2 = computeT_j(t1, P1, P2);
            t3 = computeT_j(t2, P2, P3);
        }

        Vector3 sampleRaw(float t)
        {
            Vector3 A1 = (t1 - t) / (t1 - t0) * P0 + (t - t0) / (t1 - t0) * P1;
            Vector3 A2 = (t2 - t) / (t2 - t1) * P1 + (t - t1) / (t2 - t1) * P2;
            Vector3 A3 = (t3 - t) / (t3 - t2) * P2 + (t - t2) / (t3 - t2) * P3;

            Vector3 B1 = (t2 - t) / (t2 - t0) * A1 + (t - t0) / (t2 - t0) * A2;
            Vector3 B2 = (t3 - t) / (t3 - t1) * A2 + (t - t1) / (t3 - t1) * A3;

            Vector3 C = (t2 - t) / (t2 - t1) * B1 + (t - t1) / (t2 - t1) * B2;

            return C;
        }

        Vector3 tangentRaw(float t)
        {
            Vector3 A1 = (t1 - t) / (t1 - t0) * P0 + (t - t0) / (t1 - t0) * P1;
            Vector3 A2 = (t2 - t) / (t2 - t1) * P1 + (t - t1) / (t2 - t1) * P2;
            Vector3 A3 = (t3 - t) / (t3 - t2) * P2 + (t - t2) / (t3 - t2) * P3;

            Vector3 B1 = (t2 - t) / (t2 - t0) * A1 + (t - t0) / (t2 - t0) * A2;
            Vector3 B2 = (t3 - t) / (t3 - t1) * A2 + (t - t1) / (t3 - t1) * A3;

            Vector3 dA1dt = (-1 / (t1 - t0)) * P0 + (1 / (t1 - t0)) * P1;
            Vector3 dA2dt = (-1 / (t2 - t1)) * P1 + (1 / (t2 - t1)) * P2;
            Vector3 dA3dt = (-1 / (t3 - t2)) * P2 + (1 / (t3 - t2)) * P3;

            Vector3 dB1dt = (-1 / (t2 - t0) * A1 + (t2 - t) / (t2 - t0) * dA1dt)
                + (1 / (t2 - t0) * A2 + (t - t0) / (t2 - t0) * dA2dt);

            Vector3 dB2dt = (-1 / (t3 - t1) * A2 + (t3 - t) / (t3 - t1) * dA2dt)
                + (1 / (t3 - t1) * A3 + (t - t1) / (t3 - t1) * dA3dt);

            Vector3 dCdt = (-1 / (t2 - t1) * B1 + (t2 - t) / (t2 - t1) * dB1dt)
                + (1 / (t2 - t1) * B2 + (t - t1) / (t2 - t1) * dB2dt);

            return dCdt;
        }

        public Vector3 sample(float t)
        {
            float scaledT = t * t2 + (1 - t) * t1;
            Vector3 p = sampleRaw(scaledT);
            return p;
        }

        public Vector3 sampleTangent(float t)
        {
            float scaledT = t * t2 + (1 - t) * t1;
            Vector3 p = tangentRaw(scaledT);
            return p;
        }

        const float ALPHA = 0.5f;

        float computeT_j(float ti, Vector3 pt1, Vector3 pt2)
        {
            float distance = Vector3.Distance(pt1, pt2);
            return Mathf.Pow(distance, ALPHA) + ti;
        }

        public void addSamplePointsTo(List<Vector3> points, int numPoints)
        {
            for (int i = 0; i < numPoints; i++)
            {
                float t = (float)i / numPoints;
                Vector3 p = sample(t);
                points.Add(p);
            }
        }

        public void addLastPointTo(List<Vector3> points)
        {
            points.Add(sampleRaw(t2));
        }
    } 
}