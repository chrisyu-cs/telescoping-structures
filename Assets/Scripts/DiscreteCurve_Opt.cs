using UnityEngine;
using System.Collections.Generic;

using NumericsLib;
using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

using Gurobi;

namespace Telescopes
{
    public partial class DiscreteCurve
    {
        private Matrix<double> laplacianBE;

        Vector3 AngleWrtPosition(int angleIndex, int posIndex)
        {
            int diff = Mathf.Abs(angleIndex - posIndex);
            if (diff > 1) return Vector3.zero;

            Vector3 a = discretizedPoints[angleIndex].position;
            Vector3 b = discretizedPoints[angleIndex + 1].position;
            Vector3 c = (angleIndex < 1) ? startingPoint : discretizedPoints[angleIndex - 1].position;

            Vector3 b_a = b - a;
            Vector3 c_a = c - a;
            Vector3 N = Vector3.Cross(b_a, c_a);
            float Nmag = N.magnitude;
            if (Nmag < 1e-6) return Vector3.zero;
            N.Normalize();

            Vector3 dWdb = Vector3.Cross(N, b_a) / b_a.sqrMagnitude;
            Vector3 dWdc = -Vector3.Cross(N, c_a) / c_a.sqrMagnitude;
            Vector3 dWda = -dWdb - dWdc;

            if (posIndex < angleIndex)
            {
                return dWdc;
            }
            else if (posIndex > angleIndex)
            {
                return dWdb;
            }
            else
            {
                return dWda;
            }
        }

        float DualLength(int index)
        {
            if (index < 0 || index >= discretizedPoints.Count - 1)
                throw new System.Exception("Can't take dual length of endpoint " + index);

            Vector3 prev = (index < 1) ? startingPoint : discretizedPoints[index - 1].position;
            Vector3 current = discretizedPoints[index].position;
            Vector3 next = discretizedPoints[index + 1].position;

            float length1 = Vector3.Distance(prev, current);
            float length2 = Vector3.Distance(current, next);

            return (length1 + length2) / 2;
        }

        float BendAngle(int index)
        {
            if (index < 0 || index >= discretizedPoints.Count - 1)
                throw new System.Exception("Can't take bend angle of endpoint " + index);

            Vector3 prev = (index < 1) ? startingPoint : discretizedPoints[index - 1].position;
            Vector3 current = discretizedPoints[index].position;
            Vector3 next = discretizedPoints[index + 1].position;

            Vector3 prevVec = current - prev;
            Vector3 nextVec = next - current;
            Vector3 normal = Vector3.Cross(prevVec, nextVec).normalized;

            return TelescopeUtils.AngleBetween(prevVec, nextVec, normal) * Mathf.Deg2Rad;
        }

        float TwistAngle(int index)
        {
            if (index < 0 || index >= discretizedPoints.Count - 2)
                throw new System.Exception("Can't take twist angle of endpoint " + index);

            Vector3 prev = (index < 1) ? startingPoint : discretizedPoints[index - 1].position;
            Vector3 current = discretizedPoints[index].position;
            Vector3 next1 = discretizedPoints[index + 1].position;
            Vector3 next2 = discretizedPoints[index + 2].position;

            Vector3 normal1 = Vector3.Cross(current - prev, next1 - current).normalized;
            Vector3 normal2 = Vector3.Cross(next1 - current, next2 - next1).normalized;

            Vector3 edgeVector = next1 - current;
            return Vector3.Dot(Vector3.Cross(normal1, normal2), edgeVector);
        }

        Vector3 AreaVecAtVert(int index)
        {
            // Normal is cross product of adjacent edges
            return Vector3.Cross(discretizedPoints[index].position - discretizedPoints[index - 1].position,
                discretizedPoints[index + 1].position - discretizedPoints[index].position);
        }

        Vector3 NormalAtVert(int index)
        {
            return AreaVecAtVert(index).normalized;
        }

        Vector3 DirectionalNormalWrtPosition(int normIndex, int posIndex, Vector3 u)
        {
            int diff = Mathf.Abs(normIndex - posIndex);
            if (diff > 1) return Vector3.zero;

            // Normal is cross product of adjacent edges
            Vector3 N = AreaVecAtVert(normIndex);

            float area = N.magnitude / 2;
            N.Normalize();

            int opp1, opp2;
            if (posIndex < normIndex)
            {
                opp1 = posIndex + 1;
                opp2 = posIndex + 2;
            }
            else if (posIndex == normIndex)
            {
                opp1 = posIndex - 1;
                opp2 = posIndex + 1;
            }
            else
            {
                opp1 = posIndex - 2;
                opp2 = posIndex - 1;
            }

            Vector3 e = discretizedPoints[opp2].position - discretizedPoints[opp1].position;
            Vector3 exN = Vector3.Cross(e, N);
            return Vector3.Dot(u, N) / (2 * area) * exN;
        }

        float DirectionalTorsionWrtPosition(int torsionEdgeIndex, int posIndex, Vector3 u)
        {
            if (torsionEdgeIndex < posIndex - 2) return 0;
            if (torsionEdgeIndex > posIndex + 1) return 0;
            
            Vector3 edge = discretizedPoints[torsionEdgeIndex + 1].position - discretizedPoints[torsionEdgeIndex].position;

            if (torsionEdgeIndex == posIndex - 2)
            {
                Vector3 n1 = NormalAtVert(torsionEdgeIndex);
                Vector3 dn = DirectionalNormalWrtPosition(posIndex - 1, posIndex, u);
                return Vector3.Dot(edge, Vector3.Cross(n1, dn));
            }
            else if (torsionEdgeIndex == posIndex + 1)
            {
                Vector3 n2 = NormalAtVert(torsionEdgeIndex + 1);
                Vector3 dn = DirectionalNormalWrtPosition(posIndex + 1, posIndex, u);
                return Vector3.Dot(edge, Vector3.Cross(dn, n2));
            }
            else
            {
                Vector3 n1 = NormalAtVert(torsionEdgeIndex);
                Vector3 n2 = NormalAtVert(torsionEdgeIndex + 1);

                Vector3 dn1 = DirectionalNormalWrtPosition(torsionEdgeIndex, posIndex, u);
                Vector3 dn2 = DirectionalNormalWrtPosition(torsionEdgeIndex + 1, posIndex, u);

                // If differentiating p2 wrt e23, negate the u
                // Otherwise p2 wrt e12, no negation
                float term1 = Vector3.Dot((torsionEdgeIndex == posIndex) ? -u : u, Vector3.Cross(n1, n2));
                float term2 = Vector3.Dot(edge, Vector3.Cross(dn1, n2) + Vector3.Cross(n1, dn2));
                return term1 + term2;
            }
        }

        Vector3 TorsionWrtPosition(int torsionEdgeIndex, int posIndex)
        {
            Vector3 gradient = new Vector3();
            gradient.x = DirectionalTorsionWrtPosition(torsionEdgeIndex, posIndex, Vector3.right);
            gradient.y = DirectionalTorsionWrtPosition(torsionEdgeIndex, posIndex, Vector3.up);
            gradient.z = DirectionalTorsionWrtPosition(torsionEdgeIndex, posIndex, Vector3.forward);

            return gradient;
        }

        float TorsionOfEdge(int edgeStart)
        {
            Vector3 normal1 = NormalAtVert(edgeStart);
            Vector3 normal2 = NormalAtVert(edgeStart + 1);

            Vector3 edge = discretizedPoints[edgeStart + 1].position - discretizedPoints[edgeStart].position;

            return Vector3.Dot(edge.normalized, Vector3.Cross(normal1, normal2));
        }

        void CurvaturePositionFlow(float delta)
        {
            foreach (DCurvePoint dcp in discretizedPoints)
            {
                dcp.currentGradient = Vector3.zero;
            }

            /*
            for (int i = 1; i < discretizedPoints.Count - 2; i++)
            {
                // Compute all curvature gradients

                float w_minus2 = (i > 1) ? BendAngle(i - 2) : 0;
                float w_minus1 = BendAngle(i - 1);
                float w_i = BendAngle(i);
                float w_plus1 = BendAngle(i + 1);
                float w_plus2 = (i < discretizedPoints.Count - 3) ? BendAngle(i + 2) : 0;

                float l_i = DualLength(i);

                Vector3 derivMinus1 = AngleWrtPosition(i - 1, i);
                Vector3 deriv_i = AngleWrtPosition(i, i);
                Vector3 derivPlus1 = AngleWrtPosition(i + 1, i);

                Vector3 gradient = Vector3.zero;

                if (i > 1) gradient += 2f / l_i * (w_minus1 - w_minus2) * derivMinus1;
                gradient += 2f / l_i * (w_i - w_minus1) * (deriv_i - derivMinus1);
                gradient += 2f / l_i * (w_plus1 - w_i) * (derivPlus1 - deriv_i);
                if (i < discretizedPoints.Count - 3) gradient += 2f / l_i * (w_plus2 - w_plus1) * (-derivPlus1);

                discretizedPoints[i].currentGradient = gradient;
            }*/

            
            for (int i = 1; i < discretizedPoints.Count - 2; i++)
            {
                // Compute all torsion gradients 

                float t_minus3 = (i > 3) ? TwistAngle(i - 3) : 0;
                float t_minus2 = (i > 2) ? TwistAngle(i - 2) : 0;
                float t_minus1 = (i > 1) ? TwistAngle(i - 1) : 0;
                float t_i = TwistAngle(i);
                float t_plus1 = (i < discretizedPoints.Count - 3) ? TwistAngle(i + 1) : 0;
                float t_plus2 = (i < discretizedPoints.Count - 4) ? TwistAngle(i + 2) : 0;

                float l_i = DualLength(i);
                
                Vector3 derivMinus2 = (i > 2) ? TorsionWrtPosition(i - 2, i) : Vector3.zero;
                Vector3 derivMinus1 = (i > 1) ? TorsionWrtPosition(i - 1, i) : Vector3.zero;
                Vector3 derivI = TorsionWrtPosition(i, i);
                Vector3 derivPlus1 = (i < discretizedPoints.Count - 3) ? TorsionWrtPosition(i + 1, i) : Vector3.zero;

                Vector3 gradient = Vector3.zero;

                Vector3 E1 = (i > 2) ? (t_minus2 - t_minus3) * derivMinus2 : Vector3.zero;
                Vector3 E2 = (i > 1) ? (t_minus1 - t_minus2) * (derivMinus1 - derivMinus2) : Vector3.zero;
                Vector3 E3 = (t_i - t_minus1) * (derivI - derivMinus1);
                Vector3 E4 = (i < discretizedPoints.Count - 2) ?
                    (t_plus1 - t_i) * (derivPlus1 - derivI) : Vector3.zero;
                Vector3 E5 = (i < discretizedPoints.Count - 3) ?
                    (t_plus2 - t_plus1) * (- derivPlus1) : Vector3.zero;

                gradient = 2f * (E1 + E2 + E3 + E4 + E5) / l_i;

                gradient = derivMinus2 + derivMinus1 + derivI + derivPlus1;

                discretizedPoints[i].currentGradient += gradient;
            }

            string s = "";
            float sumDiff = 0;

            for (int i = 1; i < discretizedPoints.Count - 3; i++)
            {
                float diff = TwistAngle(i + 1) - TwistAngle(i);
                s += (i == discretizedPoints.Count - 4) ? diff.ToString() : diff + ", ";
                sumDiff += Mathf.Abs(diff);
            }

            Debug.Log(s);

            float gradientMag = 0;

            foreach (DCurvePoint dcp in discretizedPoints)
            {
                dcp.position -= (delta * dcp.currentGradient);
                gradientMag += dcp.currentGradient.sqrMagnitude;
            }

            gradientMag = Mathf.Sqrt(gradientMag);
            Debug.Log("gradient magnitude = " + gradientMag + ", sum diffs = " + sumDiff);

            curvePoints = new List<Vector3>();
            curvePoints.Add(startingPoint);

            foreach (DCurvePoint dcp in discretizedPoints)
            {
                curvePoints.Add(dcp.position);
            }

            lineRender = GetComponent<LineRenderer>();
            lineRender.SetVertexCount(curvePoints.Count);
            lineRender.SetPositions(curvePoints.ToArray());

            ComputeFrenetFrames();
        }

        void CurvatureFlow(float delta)
        {
            if (discretizedPoints.Count <= 2) return;
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
            ReconstructAndAlign();
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
            ReconstructAndAlign();
        }

        void TorsionFlow(float delta)
        {
            if (discretizedPoints.Count <= 2) return;
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
            ReconstructAndAlign();
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
            ReconstructAndAlign();
        }

        public void SegmentCurvature()
        {
            List<float> curvatures = SolveCurvature(Constants.CURVATURE_SOLVE_THRESHOLD);
            
            int numSegments = 0;
            // placeholder
            float currentCurvature = 0;
            for (int i = 0; i < discretizedPoints.Count; i++)
            {
                if (i == 0 || Mathf.Abs(curvatures[i] - currentCurvature) > Constants.CURVE_SEGMENT_THRESHOLD)
                {
                    Debug.Log("difference = " + Mathf.Abs(currentCurvature - curvatures[i]));
                    currentCurvature = curvatures[i];
                    numSegments++;
                }
                else
                {
                    curvatures[i] = currentCurvature;
                }
                discretizedPoints[i].bendingAngle = currentCurvature;
            }

            Debug.Log(numSegments + " segments (out of " + discretizedPoints.Count + " possible)");

            ReconstructFromAngles();
            ComputeFrenetFrames();
            ComputeBishopFrames();

            float previous = 0;
            List<Vector3> current = null;

            float currentColor = 0;

            GameObject subcurves = new GameObject();
            subcurves.name = "subcurves";
            subcurves.transform.parent = this.transform.parent;

            // Create curves for each section of constant curvature
            for (int i = 0; i < discretizedPoints.Count; i++)
            {
                if (i == 0 || curvatures[i] != previous || i == discretizedPoints.Count - 1)
                {
                    if (current != null)
                    {
                        // Add the current point as the last one of this curve
                        current.Add(discretizedPoints[i].position);
                        // Make the curve
                        GameObject newCurveObj = new GameObject();
                        newCurveObj.name = "subcurve" + i;
                        newCurveObj.transform.parent = subcurves.transform;
                        DiscreteCurve newCurve = newCurveObj.AddComponent<DiscreteCurve>();
                        newCurve.InitFromPoints(current, segmentLength);

                        LineRenderer lr = newCurve.GetComponent<LineRenderer>();
                        lr.SetWidth(0.1f, 0.1f);
                        lr.material = lineRender.material;

                        Color c = Color.HSVToRGB(currentColor, 1, 1);
                        lr.SetColors(c, c);
                        currentColor = Mathf.Repeat(currentColor + 0.3f, 1);
                    }
                    current = new List<Vector3>();
                }
                previous = curvatures[i];
                // Add the current point again as the first one of the next curve
                current.Add(discretizedPoints[i].position);
            }

            gameObject.SetActive(false);
        }

        List<float> SolveCurvature(float degreeBounds)
        {
            GRBEnv env = new GRBEnv("curvatureOpt.log");
            GRBModel model = new GRBModel(env);

            List<float> fOrig = new List<float>();
            List<GRBVar> f_i = new List<GRBVar>();

            for (int i = 0; i < discretizedPoints.Count; i++)
            {
                // Add original curvature values to original list
                DCurvePoint point = discretizedPoints[i];
                fOrig.Add(point.bendingAngle / degreeBounds);

                // Add curvature variables
                GRBVar curv = model.AddVar(Constants.QP_LOWER_BOUND, Constants.QP_UPPER_BOUND,
                    0, GRB.CONTINUOUS, "curvature" + i);
                f_i.Add(curv);
            }

            List<GRBVar> absDiffsPlus = new List<GRBVar>();
            List<GRBVar> absDiffsMinus = new List<GRBVar>();

            for (int i = 0; i < f_i.Count - 1; i++)
            {
                GRBVar plus = model.AddVar(0, Constants.QP_UPPER_BOUND,
                    0, GRB.CONTINUOUS, "diff" + i + "+");
                GRBVar minus = model.AddVar(0, Constants.QP_UPPER_BOUND,
                    0, GRB.CONTINUOUS, "curvature" + i + "-");
                absDiffsPlus.Add(plus);
                absDiffsMinus.Add(minus);
            }

            model.Update();

            // Make constraints that define absolute values
            for (int i = 0; i < f_i.Count - 1; i++)
            {
                GRBVar vPlus = absDiffsPlus[i];
                GRBVar vMinus = absDiffsMinus[i];

                GRBLinExpr diff = f_i[i + 1] - f_i[i];
                GRBTempConstr constr = (diff == vPlus - vMinus);
                model.AddConstr(constr, "absDiff" + i);
            }

            GRBQuadExpr objective = 0;

            // Add L2 error from original function to objective
            for (int i = 0; i < f_i.Count; i++)
            {
                GRBLinExpr diff_i = (f_i[i] - fOrig[i]);
                GRBQuadExpr error_i = diff_i * diff_i;
                objective.Add(error_i);
            }
            // Add L1 diffs to objective
            for (int i = 0; i < f_i.Count - 1; i++)
            {
                objective.Add(absDiffsMinus[i] + absDiffsPlus[i]);
            }
            model.SetObjective(objective);
            model.Optimize();
            
            List<float> results = new List<float>();

            for (int i = 0; i < f_i.Count; i++)
            {
                double fi = f_i[i].Get(GRB.DoubleAttr.X);
                results.Add((float)fi * degreeBounds);
            }

            return results;
        }

        public TorsionImpulseCurve SolveImpulsesQP(List<float> arcPoints)
        {
            // ====================== BEGIN LP ======================= //
            GRBEnv env = new GRBEnv("impulseQP.log");
            GRBModel model = new GRBModel(env);

            List<GRBVar> sigma = new List<GRBVar>();

            GRBVar slope = model.AddVar(Constants.QP_LOWER_BOUND, Constants.QP_UPPER_BOUND,
                0, GRB.CONTINUOUS, "slope");

            // Make the variables.
            // We use the cumulative torsion values at each separating point as the variables,
            // because this simplifies computing the error functions later.
            for (int i = 1; i < arcPoints.Count - 1; i++)
            {
                GRBVar v = model.AddVar(Constants.QP_LOWER_BOUND, Constants.QP_UPPER_BOUND,
                    0, GRB.CONTINUOUS, "sigma" + i);
                sigma.Add(v);
            }

            List<GRBVar> absDiffsPlus = new List<GRBVar>();
            List<GRBVar> absDiffsMinus = new List<GRBVar>();

            // Made variables that will represent absolute values of diffs.
            for (int i = 0; i < sigma.Count; i++)
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
                GRBLinExpr expectedTwist;
                float segLength = arcPoints[i + 1] - arcPoints[i];
                if (i == 0) expectedTwist = slope * segLength;
                else expectedTwist = sigma[i - 1] + slope * segLength;
                GRBLinExpr diff = sigma[i] - expectedTwist;

                GRBTempConstr tempConstr = (diff == vPlus - vMinus);
                model.AddConstr(tempConstr, "diffConstr" + i);
            }

            GRBQuadExpr objective = 0;

            float cumulativeTwist = 0;

            // Add least squares error component.
            // This is the sum of all squared differences
            // between our data points and the reconstructed function.
            for (int i = 0; i < discretizedPoints.Count; i++)
            {
                cumulativeTwist += Mathf.Deg2Rad * discretizedPoints[i].twistingAngle;
                float arcPosition = (i + 1) * segmentLength;

                // Find the last arc point that comes before the sample point
                int stepNum = 0;
                for (int j = 0; j < arcPoints.Count; j++)
                {
                    if (arcPoints[j] > arcPosition) break;
                    stepNum = j;
                }
                // Find the distance past the most recent impulse point.
                float arcDistanceFromPt = arcPosition - arcPoints[stepNum];
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

            // Add L1 terms to encourage sparsity if enabled
            if (DesignerController.instance.UseSparseSolve)
            {
                for (int i = 0; i < absDiffsMinus.Count; i++)
                {
                    objective.AddTerm(1, absDiffsMinus[i]);
                    objective.AddTerm(1, absDiffsPlus[i]);
                }
            }

            // Add regularizer to favor small impulses
            double EPSILON = 0.01;

            for (int i = 1; i < sigma.Count; i++)
            {
                double arcStep = arcPoints[i + 1] - arcPoints[i];
                GRBLinExpr expectedI = sigma[i - 1] + arcStep * slope;
                GRBLinExpr impulse = sigma[i] - expectedI;

                objective.Add(EPSILON * impulse * impulse);
            }

            model.SetObjective(objective);
            model.Optimize();
            // ====================== END LP ======================= //

            for (int i = 0; i < absDiffsMinus.Count; i++)
            {
                double min = System.Math.Min(absDiffsMinus[i].Get(GRB.DoubleAttr.X),
                    absDiffsPlus[i].Get(GRB.DoubleAttr.X));
                //if (min > 1e-8) throw new System.Exception("ERROR: Absolute value was not zero");
            }

            // Read out the recommended constant torsion
            float constTorsion = (float)slope.Get(GRB.DoubleAttr.X);

            // Read out the resulting impulses by computing differences
            List<float> impulses = new List<float>();
            impulses.Add(0);
            double expectedInitial = arcPoints[1] * constTorsion;
            double initDiff = sigma[0].Get(GRB.DoubleAttr.X) - expectedInitial;
            impulses.Add((float)initDiff);

            double sumImpulses = 0;
            int numNonzeroImpulses = 0;

            for (int i = 1; i < sigma.Count; i++)
            {
                double arcStep = arcPoints[i + 1] - arcPoints[i];
                double expectedI = sigma[i - 1].Get(GRB.DoubleAttr.X) + arcStep * constTorsion;
                double diff = sigma[i].Get(GRB.DoubleAttr.X) - expectedI;
                if (System.Math.Abs(diff) > 1e-6)
                {
                    numNonzeroImpulses++;
                    sumImpulses += System.Math.Abs(diff);
                }
                // Debug.Log("Impulse " + i + " = " + diff);
                impulses.Add((float)diff);
            }
            double averageImpulse = sumImpulses / impulses.Count;

            Debug.Log(numNonzeroImpulses + " non-zero impulses (sparse QP)");

            var t = MakeTorsionImpulseCurve(impulses, arcPoints, constTorsion);
            t.SetColor(Color.cyan);

            return t;
        }

        delegate double RealFunction(double x);

        /// <summary>
        /// Compute the definite integral of a function between the two given bounds,
        /// using a trapezoidal rule approximation.
        /// </summary>
        /// <param name="intervalStart"></param>
        /// <param name="intervalEnd"></param>
        /// <param name="f"></param>
        /// <param name="numSlices"></param>
        /// <returns></returns>
        double DefiniteIntegral(double intervalStart, double intervalEnd, RealFunction f, int numSlices = 100)
        {
            double sum = 0;
            double step = (intervalEnd - intervalStart) / numSlices;
            for (int i = 0; i < numSlices; i++)
            {
                double start = intervalStart + step * i;
                double end = intervalStart + step * (i + 1);

                double average = (f(start) + f(end)) / 2;
                sum += average * step;
            }
            return sum;
        }

        delegate float TwistFunction(int index);

        double InterpolateTorsion(double arcPosition)
        {
            TwistFunction TwistOfIndex = (i => discretizedPoints[i].cumulativeTwist);
            int pointBefore = Mathf.FloorToInt((float)arcPosition / segmentLength);
            int pointAfter = Mathf.CeilToInt((float)arcPosition / segmentLength);

            if (pointBefore == pointAfter)
            {
                int i = Mathf.Clamp(pointBefore, 0, discretizedPoints.Count - 1);
                return TwistOfIndex(i);
            }
            if (pointBefore < 0) return TwistOfIndex(0);
            if (pointAfter >= discretizedPoints.Count) return TwistOfIndex(discretizedPoints.Count - 1);

            float interpFactor = (float)arcPosition - (pointBefore * segmentLength);
            float prevTwist = TwistOfIndex(pointBefore);
            float nextTwist = TwistOfIndex(pointAfter);

            return (1 - interpFactor) * prevTwist + interpFactor * nextTwist;
        }

        public TorsionImpulseCurve SolveImpulseLPSparse(List<float> arcPoints)
        {
            float ratio = 0;
            float slope = 0;
            List<float> impulses;
            List<float> sparsePoints = new List<float>(arcPoints);

            float cutoffThreshold = 0.5f;
            do
            {
                Tuple<float, List<float>> results = SolveImpulseLP(sparsePoints, makeCurve: false);
                slope = results.Item1;
                impulses = results.Item2;

                float average = 0;
                float min = Mathf.Abs(impulses[1]);
                // The first impulse is always 0 so ignore it.
                int minIndex = 0;
                for (int i = 1; i < impulses.Count; i++)
                {
                    average += Mathf.Abs(impulses[i]);
                    float nextImpulse = Mathf.Abs(impulses[i]);
                    if (nextImpulse < min)
                    {
                        min = nextImpulse;
                        minIndex = i;
                    }
                }
                average /= impulses.Count;
                ratio = min / average;
                if (ratio < cutoffThreshold) sparsePoints.RemoveAt(minIndex);
            }
            while (ratio < cutoffThreshold);

            Debug.Log(impulses.Count + " impulses (sparse LP)");

            TorsionImpulseCurve t = MakeTorsionImpulseCurve(impulses, sparsePoints, slope);
            t.SetColor(Color.magenta);
            return t;
        }

        public Tuple<float, List<float>> SolveImpulseLP(List<float> arcPoints, bool makeCurve = true)
        {
            float cumulative = 0;

            // Compute the cumulative torsion at each point
            for (int i = 0; i < discretizedPoints.Count; i++)
            {
                cumulative += Mathf.Deg2Rad * discretizedPoints[i].twistingAngle;
                discretizedPoints[i].cumulativeTwist = cumulative;
            }

            List<double> integralsPhi = new List<double>();
            List<double> integralsXPhi = new List<double>();

            TwistFunction T = (i => discretizedPoints[i].cumulativeTwist);

            for (int i = 0; i < arcPoints.Count - 1; i++)
            {
                float intervalStart = arcPoints[i];
                float intervalEnd = arcPoints[i + 1];

                RealFunction phi = InterpolateTorsion;
                RealFunction xPhi = (x => x * InterpolateTorsion(x));

                double integralPhi = DefiniteIntegral(intervalStart, intervalEnd, phi);
                double integralXPhi = DefiniteIntegral(intervalStart, intervalEnd, xPhi);

                integralsPhi.Add(integralPhi);
                integralsXPhi.Add(integralXPhi);
            }

            List<Tuple<int, int, double>> triples = new List<Tuple<int, int, double>>();
            List<double> rhsList = new List<double>();
            // Add a placeholder for now
            rhsList.Add(0);

            double sum_hi3 = 0;
            double sum_di = 0;

            for (int i = 1; i < arcPoints.Count - 1; i++)
            {
                // Compute all the relevant values
                double x_i = arcPoints[i];
                double x_i1 = arcPoints[i + 1];

                double h_i1 = x_i1 - x_i;
                double h_i2 = (x_i1 * x_i1) - (x_i * x_i);
                double h_i3 = (x_i1 * x_i1 * x_i1) - (x_i * x_i * x_i);
                double c_i = integralsPhi[i];
                double d_i = integralsXPhi[i];

                // The constraint in row i only involves the variables a (column 0) and b_i (column i).
                triples.Add(new Tuple<int, int, double>(i, 0, 0.5 * h_i2));
                triples.Add(new Tuple<int, int, double>(i, i, h_i1));
                rhsList.Add(c_i);

                sum_di += d_i;
                sum_hi3 += h_i3 / 3.0;

                // Also add the coefficient for b_i in the row 0 constraint
                triples.Add(new Tuple<int, int, double>(0, i, 0.5 * h_i2));
            }
            // Set right side of constraint 0 to be sum_{i=1}^k d_i
            rhsList[0] = sum_di;
            // Set coefficient of a in constraint 0 to be 1/3 sum_{i=1}^k h_i3
            triples.Add(new Tuple<int, int, double>(0, 0, sum_hi3));

            Matrix<double> mat = SparseMatrix.OfIndexed(arcPoints.Count - 1, arcPoints.Count - 1, triples);

            Vector<double> rhs = Vector.Build.DenseOfEnumerable(rhsList);
            Vector<double> solved = mat.Solve(rhs);

            double slope = solved[0];

            List<float> impulses = new List<float>();
            impulses.Add(0);

            for (int i = 1; i < solved.Count; i++)
            {
                double prev = (i == 1) ? 0 : solved[i - 1];
                double diff = solved[i] - prev;

                // Debug.Log("Impulse " + i + " = " + diff);
                impulses.Add((float)diff);
            }

            if (makeCurve)
            {
                TorsionImpulseCurve t = MakeTorsionImpulseCurve(impulses, arcPoints, (float)slope);
                t.SetColor(Color.magenta);
            }

            return new Tuple<float, List<float>>((float)slope, impulses);
        }
    }
}
