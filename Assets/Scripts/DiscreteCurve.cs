using UnityEngine;
using System.Collections.Generic;

using NumericsLib;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

using Telescopes;
using Gurobi;
namespace Telescopes
{
    [RequireComponent(typeof(LineRenderer))]
    public class DiscreteCurve : MonoBehaviour
    {

        private Vector3 startingPoint;
        private Vector3 startingDirection;
        private Vector3 startingBinormal;

        private List<Vector3> curvePoints;
        private List<DCurvePoint> discretizedPoints;
        private float segmentLength;

        private LineRenderer lineRender;

        private FlowMode flowMode;

        // Use this for initialization
        void Start()
        {
            flowMode = FlowMode.None;

            FrenetTracer[] fts = FindObjectsOfType<FrenetTracer>();
            foreach (var ft in fts)
            {
                ft.dCurve = this;
            }
        }

        // Compute all internal bending axes/angles from a list of points.
        public void InitFromPoints(List<Vector3> points, float segLength)
        {
            segmentLength = segLength;

            startingPoint = points[0];
            startingDirection = points[1] - points[0];
            startingDirection.Normalize();

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

                Vector3 curvatureBinormal = Vector3.Cross(previousVec, nextVec);
                if (i == 1) startingBinormal = curvatureBinormal;

                // Compute bending angles (discrete curvature).
                float bendAngle = Mathf.Rad2Deg * Mathf.Acos(Vector3.Dot(previousVec, nextVec));
                // Compute twisting angles (discrete torsion).
                float twistAngle;
                // Compute twist angles as we go along.
                // The first vertex is considered to have no twist.
                if (i == 1)
                    twistAngle = 0;
                else
                {
                    twistAngle = TelescopeUtils.AngleBetween(prevBinormal, curvatureBinormal, previousVec);
                }

                prevBinormal = curvatureBinormal;

                DCurvePoint dcp = new DCurvePoint(curvatureBinormal.normalized,
                    bendAngle, twistAngle, segmentLength);

                discretizedPoints.Add(dcp);
            }

            ReconstructFromAngles();
            ComputeFrenetFrames();
            ComputeBishopFrames();
        }

        void ReconstructFromAngles()
        {
            curvePoints = new List<Vector3>();

            Vector3 currentPoint = startingPoint;
            curvePoints.Add(currentPoint);
            Vector3 currentDir = startingDirection;

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

        void ComputeFrenetFrames()
        {
            for (int i = 0; i < discretizedPoints.Count; i++)
            {
                DCurvePoint dcp = discretizedPoints[i];
                if (i == 0)
                {
                    Vector3 prevPos = startingPoint;
                    dcp.ComputeFrenet(prevPos, discretizedPoints[i + 1]);
                }
                else if (i == discretizedPoints.Count - 1)
                {
                    dcp.frenetFrame = discretizedPoints[i - 1].frenetFrame;
                }
                else
                {
                    DCurvePoint prevPoint = discretizedPoints[i - 1];
                    DCurvePoint nextPoint = discretizedPoints[i + 1];
                    dcp.ComputeFrenet(prevPoint, nextPoint);
                }
            }
        }

        void ComputeBishopFrames()
        {
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

        private Matrix<double> laplacianBE;

        void CurvatureFlow(float delta)
        {
            if (laplacianBE == null)
            {
                Matrix<double> laplacian = NumericalUtils.SpaceCurveLaplacian(discretizedPoints.Count);
                laplacianBE = NumericalUtils.LaplacianToImplicitEuler(laplacian, delta);
            }
            List<float> rotationAngles = discretizedPoints.ConvertAll<float>(DCurvePoint.ToBendAngle);
            Vector<double> rhs = NumericalUtils.VectorFromList(rotationAngles);
            Vector<double> solved = laplacianBE.Solve(rhs);

            for (int i = 0; i < rotationAngles.Count; i++)
            {
                discretizedPoints[i].bendingAngle = (float)solved[i];
            }
            ReconstructFromAngles();
        }

        void CurvatureToAverage()
        {
            float totalAngle = 0;
            foreach (DCurvePoint dcp in discretizedPoints)
            {
                totalAngle += dcp.bendingAngle;
            }
            float averageAngle = totalAngle / discretizedPoints.Count;
            foreach (DCurvePoint dcp in discretizedPoints)
            {
                dcp.bendingAngle = averageAngle;
            }
            ReconstructFromAngles();
        }

        void TorsionFlow(float delta)
        {
            if (laplacianBE == null)
            {
                Matrix<double> laplacian = NumericalUtils.SpaceCurveLaplacian(discretizedPoints.Count);
                laplacianBE = NumericalUtils.LaplacianToImplicitEuler(laplacian, delta);
            }
            List<float> twistAngles = discretizedPoints.ConvertAll<float>(DCurvePoint.ToTwistAngle);
            Vector<double> rhs = NumericalUtils.VectorFromList(twistAngles);
            Vector<double> solved = laplacianBE.Solve(rhs);

            for (int i = 0; i < twistAngles.Count; i++)
            {
                discretizedPoints[i].twistingAngle = (float)(solved[i]);
            }
            ReconstructFromAngles();
        }

        void TorsionToAverage()
        {
            float totalAngle = 0;
            for (int i = 1; i < discretizedPoints.Count; i++)
            {
                totalAngle += discretizedPoints[i].twistingAngle;
            }
            float averageAngle = totalAngle / (discretizedPoints.Count - 1);
            for (int i = 1; i < discretizedPoints.Count; i++)
            {
                discretizedPoints[i].twistingAngle = averageAngle;
            }
            ReconstructFromAngles();
        }

        // Update is called once per frame
        void Update()
        {

            if (Input.GetKeyDown("l"))
            {
                bool isShiftKeyDown = Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift);
                if (isShiftKeyDown)
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

            else if (Input.GetKeyDown("k"))
            {
                bool isShiftKeyDown = Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift);
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

            else if (Input.GetKeyDown("o"))
            {
                SolveImpulses(10);
            }

            if (flowMode == FlowMode.CurvatureFlow)
            {
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

        public float ArcLength
        {
            get { return discretizedPoints.Count * segmentLength; }
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

        public void SolveImpulses(int numSegments)
        {
            GRBEnv env = new GRBEnv("impulseQP.log");
            GRBModel model = new GRBModel(env);

            List<GRBVar> sigma = new List<GRBVar>();

            float arcStep = ArcLength / numSegments;

            GRBVar slope = model.AddVar(Constants.QP_LOWER_BOUND, Constants.QP_UPPER_BOUND,
                0, GRB.CONTINUOUS, "slope");

            // Make the variables.
            // We use the cumulative torsion values at each separating point as the variables,
            // because this simplifies computing the error functions later.
            for (int i = 1; i < numSegments; i++)
            {
                GRBVar v = model.AddVar(Constants.QP_LOWER_BOUND, Constants.QP_UPPER_BOUND,
                    0, GRB.CONTINUOUS, "sigma" + i);
                sigma.Add(v);
            }

            List<GRBVar> absDiffsPlus = new List<GRBVar>();
            List<GRBVar> absDiffsMinus = new List<GRBVar>();

            // Made variables that will represent absolute values of diffs.
            for (int i = 1; i < sigma.Count; i++)
            {
                GRBVar vPlus = model.AddVar(0, Constants.QP_UPPER_BOUND,
                    0, GRB.CONTINUOUS, "diff" + i + "+");
                GRBVar vMinus = model.AddVar(0, Constants.QP_UPPER_BOUND,
                    0, GRB.CONTINUOUS, "diff" + i + "-");
                absDiffsPlus.Add(vPlus);
                absDiffsMinus.Add(vMinus);
            }

            model.Update();

            // Add constraints that will define the plus and minus vars
            // as absolute values.
            for (int i = 0; i < absDiffsMinus.Count; i++)
            {
                GRBVar vPlus = absDiffsPlus[i];
                GRBVar vMinus = absDiffsMinus[i];
                GRBLinExpr diff = sigma[i + 1] - sigma[i];

                GRBTempConstr tempConstr = (diff == vPlus - vMinus);
                model.AddConstr(tempConstr, "diffConstr" + i);

                GRBVar[] vars = new GRBVar[] { vPlus, vMinus };
                double[] weights = { 1, 1 };
                model.AddSOS(vars, weights, GRB.SOS_TYPE1);
            }

            GRBQuadExpr objective = 0;

            float cumulativeTwist = 0;

            // Add least squares error component.
            // This is the sum of all squared differences
            // between our data points and the reconstructed function.
            for (int i = 0; i < discretizedPoints.Count; i++)
            {
                cumulativeTwist += Mathf.Deg2Rad * discretizedPoints[i].twistingAngle;
                float arcPosition = i * segmentLength;
                int stepNum = Mathf.FloorToInt(arcPosition / arcStep);
                float arcDistanceFromPt = arcPosition - (stepNum * arcStep);
                // Implicitly sigma_0 = 0, so the error contribution here
                // is just the difference from the segments starting at 0.
                GRBQuadExpr error;
                GRBLinExpr sigma_slope;
                if (stepNum == 0)
                {
                    sigma_slope = 0 + slope * arcDistanceFromPt;
                }
                // Otherwise we use sigma_j as the starting point.
                else
                {
                    // We didn't include sigma_0 in the list, 
                    // so actually sigma_1 = sigma[0].
                    int j = stepNum - 1;
                    sigma_slope = sigma[j] + slope * arcDistanceFromPt;
                }
                error = (cumulativeTwist - sigma_slope) * (cumulativeTwist - sigma_slope);
                objective.Add(error);
            }

            for (int i = 0; i < absDiffsMinus.Count; i++)
            {
                objective.AddTerm(1, absDiffsMinus[i]);
                objective.AddTerm(1, absDiffsPlus[i]);
            }

            model.SetObjective(objective);
            model.Optimize();

            // Read out the resulting impulses by computing differences
            List<float> impulses = new List<float>();
            impulses.Add(0);
            impulses.Add((float)sigma[0].Get(GRB.DoubleAttr.X));

            for (int i = 1; i < sigma.Count; i++)
            {
                double diff = sigma[i].Get(GRB.DoubleAttr.X) - sigma[i - 1].Get(GRB.DoubleAttr.X);
                impulses.Add((float)diff);
            }

            for (int i = 0; i < impulses.Count; i++)
            {
                Debug.Log("Impulse (radians) " + i + " = " + impulses[i]);
            }

            Debug.Log(slope.Get(GRB.StringAttr.VarName) + " = " + slope.Get(GRB.DoubleAttr.X));

            Debug.Log("Objective value: " + model.Get(GRB.DoubleAttr.ObjVal));
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
        public float segmentLength;

        public float twistingAngle;

        public Vector3 position;

        public Vector3 tangent
        {
            get { return frenetFrame.T; }
        }

        public OrthonormalFrame frenetFrame;
        public OrthonormalFrame bishopFrame;

        public DCurvePoint(Vector3 dir, float bendAngle, float twistAngle, float len)
        {
            binormal = dir;
            bendingAngle = bendAngle;
            twistingAngle = twistAngle;
            segmentLength = len;
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