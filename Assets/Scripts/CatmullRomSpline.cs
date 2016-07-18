using UnityEngine;
using System.Collections.Generic;

[RequireComponent(typeof(LineRenderer))]
public class CatmullRomSpline : MonoBehaviour {

    public List<Vector3> points;
    private List<CatmullRomSegment> segments;
    private LineRenderer lineRender;

    public int pointsPerSegment = 20;

    private bool needsUpdate = false;
    public bool randomize = true;

	// Use this for initialization
	void Start () {
        if (points.Count < 4) return;

        if (randomize)
        {
            for (int i = 0; i < points.Count; i++)
            {
                points[i] = 2f * Random.insideUnitSphere;
                points[i] += 2f * Vector3.up;
            }
        }

        segments = new List<CatmullRomSegment>();

        for (int i = 0; i < points.Count - 3; i++)
        {
            Vector3 p0 = points[i];
            Vector3 p1 = points[i + 1];
            Vector3 p2 = points[i + 2];
            Vector3 p3 = points[i + 3];

            segments.Add(new CatmullRomSegment(p0, p1, p2, p3));
        }

        lineRender = GetComponent<LineRenderer>();

        updateRenderPoints();
        createSpheres();
	}
	
    void updateRenderPoints()
    {
        List<Vector3> points = new List<Vector3>();
        for (int i = 0; i < segments.Count; i++)
        {
            segments[i].addSamplePointsTo(points, pointsPerSegment);
        }
        segments[segments.Count - 1].addLastPointTo(points);

        lineRender.SetVertexCount(points.Count);
        lineRender.SetPositions(points.ToArray());

        needsUpdate = false;
    }

    void createSpheres()
    {
        foreach (var p in points)
        {
            GameObject obj = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            obj.name = "splineMarker";
            obj.transform.localScale = new Vector3(0.1f, 0.1f, 0.1f);
            obj.transform.position = p;
            obj.transform.parent = transform;
        }
    }
    
    Vector3 sample(float t)
    {
        int segNum = Mathf.FloorToInt(t);
        if (segNum >= segments.Count)
        {
            return segments[segments.Count - 1].sample(1);
        }
        float segT = t - segNum;
        return segments[segNum].sample(segT);
    }

    float searchNextPoint(float startT, float segmentLength)
    {
        float maxT = segments.Count;
        float diff = 0.1f;
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
            Debug.Log("MAX ITERATIONS EXCEEDED");
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
            allPoints.Add(sample(nextT));
            curT = nextT;
        }

        return allPoints;
    }

	// Update is called once per frame
	void Update () {
	    if (Input.GetKeyDown("p"))
        {
            float segLength = 0.1f;

            List<Vector3> discretePoints = discretizeSpline(segLength);

            lineRender.SetVertexCount(discretePoints.Count);
            lineRender.SetPositions(discretePoints.ToArray());

            
            GameObject discretized = new GameObject();
            
            discretized.name = "DiscretizedCurve";
            DiscreteCurve dCurve = discretized.AddComponent<DiscreteCurve>();
            dCurve.InitFromPoints(discretePoints, segLength);

            LineRenderer lr = dCurve.GetComponent<LineRenderer>();
            lr.SetWidth(0.1f, 0.1f);
            lr.material = lineRender.material;

            gameObject.SetActive(false);
        }
	}
}

public class CatmullRomSegment
{
    public Vector3 P0, P1, P2, P3;
    private float t0, t1, t2, t3;

    public CatmullRomSegment(Vector3 v0, Vector3 v1, Vector3 v2, Vector3 v3)
    {
        // Set points
        P0 = v0;
        P1 = v1;
        P2 = v2;
        P3 = v3;
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

    public Vector3 sample(float t)
    {
        float scaledT = t * t2 + (1 - t) * t1;
        Vector3 p = sampleRaw(scaledT);
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