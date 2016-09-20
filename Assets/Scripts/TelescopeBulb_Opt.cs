using UnityEngine;

using System;
using System.Collections.Generic;

using NumericsLib;
using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

using Gurobi;

namespace Telescopes
{
    public partial class TelescopeBulb
    {
        void Rescale(float newRadius)
        {
            float scaleFactor = newRadius / Radius;
            float radiusDiff = newRadius - Radius;

            // Offset us from the parent segment so that we are in contact again.
            if (parentSegment)
            {
                Vector3 normal = parentSegment.WorldEndTangent();
                Vector3 offset = -radiusDiff * normal;

                transform.position = transform.position + offset;
            }

            // Offset child segments from this bulb so that they are in contact.
            foreach (TelescopingSegment seg in childSegments)
            {
                float childRadius = seg.shells[0].radius;
                float childOffset = Mathf.Sqrt(newRadius * newRadius - childRadius * childRadius);

                if (seg.Reversed)
                {
                    TelescopingShell shell = seg.shells[0];
                    Vector3 shellOffset = shell.transform.rotation * shell.getLocalLocationAlongPath(1);

                    Vector3 localPos = seg.LocalContactTangent() * childOffset;
                    Vector3 desiredShellEnd = transform.rotation * localPos + transform.position;

                    Vector3 shellBaseWorld = shell.transform.position;
                    Vector3 currentShellEnd = shellBaseWorld + shellOffset;

                    Vector3 worldOffset = desiredShellEnd - currentShellEnd;

                    seg.transform.position += worldOffset;// + shellOffset;
                }

                else
                {
                    seg.transform.localPosition = seg.LocalContactTangent() * childOffset;
                }

            }

            // Rescale the bulb itself
            Radius = newRadius;
        }

        Vector3 CartesianOfSpherical(float theta, float phi)
        {
            float x = Mathf.Sin(theta) * Mathf.Cos(phi);
            float y = Mathf.Sin(theta) * Mathf.Sin(phi);
            float z = Mathf.Cos(theta);

            return new Vector3(x, y, z);
        }

        public void OptimizeCollisions()
        {
            bool allOK = CollisionIteration();

            if (allOK)
            {
                Debug.Log("Done");
            }
            else
            {
                Rescale(Radius * 1.05f);
            }
        }

        void RotateChildSegment(TelescopingSegment seg, Quaternion localRotation)
        {
            Vector3 localPos = localRotation * seg.transform.localPosition;
            seg.transform.localPosition = localPos;

            Quaternion localRot = localRotation * seg.shells[0].transform.localRotation;
            seg.shells[0].transform.localRotation = localRot;
        }

        bool CollisionIteration()
        {
            bool allOK = true;

            if (parentSegment)
            {
                Vector3 parentTangent = Quaternion.Inverse(transform.rotation) * parentSegment.WorldEndTangent();
                float parentRadius = parentSegment.shells[parentSegment.shells.Count - 1].radius;

                float parentTheta = Mathf.Acos(parentRadius / Radius);

                // Project all segments away from the parent segment
                foreach (TelescopingSegment seg in childSegments)
                {
                    float childRadius = seg.shells[0].radius;
                    Vector3 childTangent = seg.LocalContactTangent();
                    float childTheta = Mathf.Asin(childRadius / Radius);
                    float angleBetween = Mathf.Acos(Vector3.Dot(parentTangent, childTangent));

                    /*
                    Vector3 worldParent = transform.rotation * parentTangent;
                    Vector3 worldChild = transform.rotation * childTangent;

                    List<Vector3> parentPts = new List<Vector3>();
                    parentPts.Add(transform.position);
                    parentPts.Add(transform.position + 5f * worldParent);

                    List<Vector3> childPts = new List<Vector3>();
                    childPts.Add(transform.position);
                    childPts.Add(transform.position + 5f * worldChild);

                    TelescopeUtils.DisplayLine(parentPts);
                    TelescopeUtils.DisplayLine(childPts);
                    */

                    if (parentTheta + childTheta > angleBetween)
                    {
                        allOK = false;

                        Debug.Log("Angle to parent = " + angleBetween + ", individual angles "
                            + parentTheta + " + " + childTheta + " = " + (parentTheta + childTheta));

                        float angleDiff = parentTheta + childTheta - angleBetween;
                        angleDiff *= Mathf.Rad2Deg;

                        Vector3 normal = Vector3.Cross(parentTangent, childTangent);
                        Quaternion rot = Quaternion.AngleAxis(angleDiff, normal);

                        RotateChildSegment(seg, rot);
                    }
                }
            }

            // Project all pairs of child segments
            for (int i = 0; i < childSegments.Count; i++)
            {
                for (int j = i + 1; j < childSegments.Count; j++)
                {
                    Vector3 tangent1 = childSegments[i].LocalContactTangent();
                    Vector3 tangent2 = childSegments[j].LocalContactTangent();

                    float radius1 = childSegments[i].shells[0].radius;
                    float radius2 = childSegments[j].shells[0].radius;

                    float angleBetween = Mathf.Acos(Vector3.Dot(tangent1, tangent2));
                    float theta1 = Mathf.Asin(radius1 / Radius);
                    float theta2 = Mathf.Asin(radius2 / Radius);
                    
                    if (theta1 + theta2 > angleBetween)
                    {
                        allOK = false;

                        Debug.Log("Angle between = " + angleBetween + ", individual angles "
                            + theta1 + " + " + theta2 + " = " + (theta1 + theta2));

                        float angleDiff = theta1 + theta2 - angleBetween;
                        angleDiff *= Mathf.Rad2Deg;

                        Vector3 normal = Vector3.Cross(tangent1, tangent2);
                        Quaternion rot1 = Quaternion.AngleAxis(-angleDiff / 2, normal);
                        Quaternion rot2 = Quaternion.AngleAxis(angleDiff / 2, normal);

                        RotateChildSegment(childSegments[i], rot1);
                        RotateChildSegment(childSegments[j], rot2);
                    }
                }
            }

            return allOK;
        }

        public void OptimizeCollisionsSOCP()
        {
            // Read out initial values of variables
            float initialRadius = Radius;

            List<float> childRadii = new List<float>();
            List<Vector3> childBases = new List<Vector3>();

            Debug.Log("Bulb " + name + " with radius " + initialRadius);

            foreach (TelescopingSegment seg in childSegments)
            {
                float childRadius = seg.shells[0].radius;
                childRadii.Add(childRadius);

                Vector3 childTangent = seg.LocalContactTangent();
                float baseDistance = Mathf.Sqrt(initialRadius * initialRadius - childRadius * childRadius);
                Vector3 childBase = childTangent * baseDistance;
                childBases.Add(childBase);

                Debug.Log("Child segment " + seg.name + " with radius " + childRadius + ", base position " + childBase);
            }

            // Set up Gurobi
            GRBEnv env = new GRBEnv("bulbOpt.log");
            GRBModel model = new GRBModel(env);

            List<GRBVar> radiiVars = new List<GRBVar>();
            List<GRBVar> baseXVars = new List<GRBVar>();
            List<GRBVar> baseYVars = new List<GRBVar>();
            List<GRBVar> baseZVars = new List<GRBVar>();

            float maxRadius = 2 * initialRadius;

            // Make variable for sphere radius
            GRBVar sphereRadius = model.AddVar(initialRadius, maxRadius, 0, GRB.CONTINUOUS, "sphereRadius");

            // Make variables for child base positions:
            // 3 for each child segment
            for (int i = 0; i < childSegments.Count; i++)
            {
                GRBVar cylBaseX = model.AddVar(-maxRadius, maxRadius, 0, GRB.CONTINUOUS, "baseX" + i);
                GRBVar cylBaseY = model.AddVar(-maxRadius, maxRadius, 0, GRB.CONTINUOUS, "baseY" + i);
                GRBVar cylBaseZ = model.AddVar(-maxRadius, maxRadius, 0, GRB.CONTINUOUS, "baseZ" + i);

                baseXVars.Add(cylBaseX);
                baseYVars.Add(cylBaseY);
                baseZVars.Add(cylBaseZ);
            }

            model.Update();

            // Create constraint on base radius - must be greater than all cylinder radii
            for (int i = 0; i < childSegments.Count; i++)
            {
                GRBTempConstr sphereConstr = (sphereRadius >= childRadii[i]);
                model.AddConstr(sphereConstr, "sphereConstr" + i);
            }

            // Create constraints on base positions - must be in contact with sphere
            for (int i = 0; i < childSegments.Count; i++)
            {
                GRBQuadExpr radiusExpr = sphereRadius * sphereRadius - childRadii[i] * childRadii[i];
                GRBQuadExpr normExpr = baseXVars[i] * baseXVars[i]
                    + baseYVars[i] * baseYVars[i]
                    + baseZVars[i] * baseZVars[i];
                GRBTempConstr normConstr = (normExpr <= radiusExpr);

                model.AddQConstr(normConstr, "baseNormConstr" + i);
            }

            // Create pairwise non-collision constraints.
            for (int i = 0; i < childSegments.Count; i++)
            {
                for (int j = i + 1; j < childSegments.Count; j++)
                {
                    /*
                    float sumRadii = childRadii[i] + childRadii[j];

                    GRBLinExpr diffX = baseXVars[i] - baseXVars[j];
                    GRBLinExpr diffY = baseYVars[i] - baseYVars[j];
                    GRBLinExpr diffZ = baseZVars[i] - baseZVars[j];

                    GRBQuadExpr distSq = diffX * diffX + diffY * diffY + diffZ * diffZ;

                    GRBTempConstr collConstr = (distSq >= sumRadii * sumRadii);
                    model.AddQConstr(collConstr, "collisionConstr" + i);*/
                }
            }

            // Create collision constraints with parent.
            if (parentSegment)
            {
                float parentRadius = parentSegment.shells[0].radius;

                Vector3 parentTangent = Quaternion.Inverse(transform.rotation) * parentSegment.WorldEndTangent();
                float baseDistance = Mathf.Sqrt(initialRadius * initialRadius - parentRadius * parentRadius);
                Vector3 parentBase = parentTangent * baseDistance;

                for (int i = 0; i < childSegments.Count; i++)
                {
                    float sumRadii = childRadii[i] + parentRadius;

                    GRBLinExpr diffX = baseXVars[i] - parentBase.x;
                    GRBLinExpr diffY = baseYVars[i] - parentBase.y;
                    GRBLinExpr diffZ = baseZVars[i] - parentBase.z;

                    GRBQuadExpr distSq = diffX * diffX + diffY * diffY + diffZ * diffZ;

                    GRBTempConstr collConstr = (distSq >= sumRadii * sumRadii);
                    //model.AddQConstr(collConstr, "parentCollisionConstr" + i);
                }
            }

            // Start formulating objective.
            // Add difference in sphere radius.
            GRBLinExpr radiusDiff = sphereRadius - initialRadius;
            GRBQuadExpr radiusDiffSq = radiusDiff * radiusDiff;

            // Add distance moved by cylinder base positions.
            for (int i = 0; i < childSegments.Count; i++)
            {
                GRBLinExpr diffX = baseXVars[i] - childBases[i].x;
                GRBLinExpr diffY = baseYVars[i] - childBases[i].y;
                GRBLinExpr diffZ = baseZVars[i] - childBases[i].z;

                GRBQuadExpr distSq = diffX * diffX + diffY * diffY + diffZ * diffZ;
                radiusDiffSq.Add(distSq);
            }

            model.SetObjective(radiusDiffSq);

            model.Optimize();

            // Read base sphere radius.
            float newRadius = (float)sphereRadius.Get(GRB.DoubleAttr.X);

            Debug.Log("Sphere radius " + initialRadius + " => " + newRadius);

            // Read back cylinder base positions.
            for (int i = 0; i < childSegments.Count; i++)
            {
                float newX = (float)baseXVars[i].Get(GRB.DoubleAttr.X);
                float newY = (float)baseYVars[i].Get(GRB.DoubleAttr.X);
                float newZ = (float)baseZVars[i].Get(GRB.DoubleAttr.X);

                Vector3 cylBase = new Vector3(newX, newY, newZ);

                Debug.Log("Base " + i + ": " + childBases[i] + " => " + cylBase);
            }
        }
    }
}
