using UnityEngine;
using System.Collections.Generic;

using NumericsLib;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

[RequireComponent(typeof(LineRenderer))]
public class DiscreteCurve : MonoBehaviour {

    private Vector3 startingPoint;
    private Vector3 startingDirection;

    private List<Vector3> curvePoints;
    private List<Vector3> rotationDirections;
    private List<float> rotationAngles;
    private float segmentLength;

    private LineRenderer lineRender;

    private bool doCurvatureFlow;

	// Use this for initialization
	void Start () {
        doCurvatureFlow = false;
	}

    // Compute all internal bending axes/angles from a list of points.
    public void InitFromPoints(List<Vector3> points, float segLength)
    {
        segmentLength = segLength;

        startingPoint = points[0];
        startingDirection = points[1] - points[0];
        startingDirection.Normalize();

        rotationDirections = new List<Vector3>();
        rotationAngles = new List<float>();

        // We need to store bending angles / directions of all the interior
        // vertices of the curve (but not the endpoints).
        for (int i = 1; i < points.Count - 1; i++)
        {
            Vector3 previousVec = points[i] - points[i - 1];
            Vector3 nextVec = points[i + 1] - points[i];

            previousVec.Normalize();
            nextVec.Normalize();

            Vector3 curvatureBinormal = Vector3.Cross(previousVec, nextVec);

            rotationDirections.Add(curvatureBinormal.normalized);

            float angle = Mathf.Rad2Deg * Mathf.Acos(Vector3.Dot(previousVec, nextVec));
            rotationAngles.Add(angle);
        }

        ReconstructFromAngles();
    }

    void ReconstructFromAngles()
    {
        curvePoints = new List<Vector3>();

        Vector3 currentPoint = startingPoint;
        curvePoints.Add(currentPoint);
        Vector3 currentDir = startingDirection;

        currentPoint += currentDir * segmentLength;
        curvePoints.Add(currentPoint);

        // Rotate the direction about the current binormal by the given angle,
        // and then offset to reach the next point.
        for (int i = 0; i < rotationDirections.Count; i++)
        {
            Quaternion nextRot = Quaternion.AngleAxis(rotationAngles[i], rotationDirections[i]);
            currentDir = nextRot * currentDir;
            currentPoint += currentDir * segmentLength;
            curvePoints.Add(currentPoint);
        }

        SetupLineRenderer();
    }

    void SetupLineRenderer()
    {
        lineRender = GetComponent<LineRenderer>();
        lineRender.SetVertexCount(curvePoints.Count);
        lineRender.SetPositions(curvePoints.ToArray());
    }

    private Matrix<double> laplacianBE;

    void CurvatureFlow(float delta)
    {
        if (laplacianBE == null)
        {
            Matrix<double> laplacian = NumericalUtils.SpaceCurveLaplacian(rotationAngles.Count);
            laplacianBE = NumericalUtils.LaplacianToImplicitEuler(laplacian, delta);
        }
        Vector<double> rhs = NumericalUtils.VectorFromList(rotationAngles);
        Vector<double> solved = laplacianBE.Solve(rhs);

        for (int i = 0; i < rotationAngles.Count; i++)
        {
            rotationAngles[i] = (float)solved[i];
        }
        ReconstructFromAngles();
    }

    void CurvatureFlowFE(float delta)
    {
        // For testing, just use forward Euler.

        List<float> laplacian = new List<float>();

        // Compute laplacian: average of neighboring vertices
        for (int i = 0; i < rotationAngles.Count; i++)
        {
            if (i == 0)
            {
                laplacian.Add((rotationAngles[i] + rotationAngles[i + 1]) / 2);
            }
            else if (i == rotationAngles.Count - 1)
            {
                laplacian.Add((rotationAngles[i] + rotationAngles[i - 1]) / 2);
            }
            else
            {
                float diff = (rotationAngles[i + 1] + rotationAngles[i - 1]) / 2;
                laplacian.Add(diff);
            }
        }

        // Take one time step
        for (int i = 0; i < rotationAngles.Count; i++)
        {
            rotationAngles[i] += delta * laplacian[i];
        }

        ReconstructFromAngles();
    }

    // Update is called once per frame
    void Update () {

        if (Input.GetKeyDown("l"))
        {
            doCurvatureFlow = !doCurvatureFlow;
        }

	    if (doCurvatureFlow)
        {
            CurvatureFlow(1f);
        }
	}
}
