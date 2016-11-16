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
            float radiusDiff = newRadius - Radius;

            // Offset us from the parent segment so that we are in contact again.
            if (parentSegment)
            {
                Vector3 normal = parentSegment.WorldEndTangent();
                Vector3 offset = -radiusDiff * normal;

                transform.position = transform.position + offset;
            }

            // Offset child segments from this bulb so that they are in contact.
            foreach (TelescopeSegment seg in childSegments)
            {
                float childRadius = seg.shells[0].radius;
                float childOffset = Mathf.Sqrt(newRadius * newRadius - childRadius * childRadius);

                if (seg.Reversed)
                {
                    TelescopeShell shell = seg.shells[0];
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

        void RotateChildSegment(TelescopeSegment seg, Quaternion localRotation)
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
                foreach (TelescopeSegment seg in childSegments)
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
    }
}
