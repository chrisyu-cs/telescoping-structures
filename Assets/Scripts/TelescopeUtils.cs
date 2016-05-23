using UnityEngine;
using System.Collections.Generic;

namespace Telescopes
{
    public class TelescopeUtils
    {
        public static Vector3 translateAlongCircle(float curvatureAmount, float arcLength)
        {
            if (curvatureAmount > 1e-6)
            {
                float curvatureRadius = 1f / curvatureAmount;

                // Start at the bottom of the circle.
                float baseAngle = 3 * Mathf.PI / 2; // + baseRadians;

                // Compute how many radians we moved.
                float radians = arcLength * curvatureAmount;
                float finalAngle = baseAngle + radians;

                // Compute on the circle centered at (0, 0, 0),
                // and then add (0, r, 0).
                Vector3 center = Vector3.up * curvatureRadius;
                Vector3 displacement = Vector3.zero;
                displacement.z = Mathf.Cos(finalAngle) * curvatureRadius;
                displacement.y = Mathf.Sin(finalAngle) * curvatureRadius;
                displacement += center;
                return displacement;
            }
            else
            {
                Vector3 displacement = (arcLength /* + baseRadians */) * Vector3.forward;
                return displacement;
            }
        }

        public static Quaternion rotateAlongCircle(float curvatureAmount, float arcLength)
        {
            if (curvatureAmount < 1e-6) return Quaternion.identity;
            // Compute how many radians we moved.
            float radians = arcLength * curvatureAmount; // + baseRadians;

            // Now rotate the forward vector by that amount.
            Vector3 axisOfRotation = Vector3.right;
            float degrees = radians / Mathf.Deg2Rad;
            Quaternion rotation = Quaternion.AngleAxis(-degrees, axisOfRotation);
            return rotation;
        }

        public static Vector3 childBasePosition(TelescopeParameters parent, TelescopeParameters child)
        {
            Vector3 translationToBase = translateAlongCircle(parent.curvature, parent.length);
            Quaternion rotationToBase = rotateAlongCircle(parent.curvature, parent.length);
            Vector3 translationBackwards = translateAlongCircle(child.curvature, -child.length);

            translationToBase = translationToBase + (rotationToBase * translationBackwards);
            return translationToBase;
        }

        public static Quaternion childBaseRotation(TelescopeParameters parent, TelescopeParameters child)
        {
            Quaternion rotationToBase = rotateAlongCircle(parent.curvature, parent.length);
            Quaternion rotationBack = rotateAlongCircle(child.curvature, -child.length);
            return rotationBack * rotationToBase;
        }

        public static void growParentToChild(TelescopeParameters parent, TelescopeParameters child, bool shrinkFit=false)
        {
            // Compute the coordinates of the child's starting position,
            // in the parent's coordinate frame.
            Vector3 childBasePos = childBasePosition(parent, child);
            Quaternion childBaseRot = childBaseRotation(parent, child);

            Vector3 childOutward = childBaseRot * Vector3.down;

            // Compute the two inner corners of the child, when nested inside its
            // parent; these are the corners we want to bounds check.
            Vector3 retractedInner = childBasePos - (child.radius * childOutward);
            Vector3 retractedOuter = childBasePos + (child.radius * childOutward);

            // If the parent has no curvature, then the required width is just the
            // max absolute y value of the child.
            float finalRadius;

            if (parent.curvature < 1e-6)
            {
                finalRadius = Mathf.Max(Mathf.Abs(retractedInner.y), Mathf.Abs(retractedOuter.y)) + parent.thickness;
                if (!shrinkFit) finalRadius = Mathf.Max(finalRadius, parent.radius);
            }

            // Otherwise we need to compute the distance from the center of rotation to the
            // corners, and take their max distance from the center line.
            else
            {
                // Fact: The centers of rotation for the parent and the extended child both lie
                // on the line (in the ZY plane) defined by the parent's center and its far face.
                float parentRadius = 1f / parent.curvature;
                Vector3 centerOfRotation = new Vector3(0, parentRadius, 0);

                // Now we can determine what width we need to 'fatten' the parent by.
                float innerDistance = Vector3.Distance(retractedInner, centerOfRotation);
                float outerDistance = Vector3.Distance(retractedOuter, centerOfRotation);
                float innerDiff = Mathf.Abs(innerDistance - parentRadius);
                float outerDiff = Mathf.Abs(outerDistance - parentRadius);

                finalRadius = Mathf.Max(innerDiff, outerDiff) + parent.thickness;
                if (!shrinkFit) finalRadius = Mathf.Max(finalRadius, parent.radius);
            }
            float widthChange = finalRadius - parent.radius;

            parent.radius = finalRadius;
            // parent.thickness += widthChange;
        }

        public static void growChainToFit(List<TelescopeParameters> parameters, bool shrinkFit = false)
        {
            // Make a pass in reverse that grows each parent so that it is large enough
            // to contain its child.
            for (int i = parameters.Count - 1; i > 0; i--)
            {
                TelescopeUtils.growParentToChild(parameters[i - 1], parameters[i], shrinkFit);
            }

        }

        public static TelescopeParameters shrinkChildToParent(TelescopeParameters parent, TelescopeParameters child)
        {
            Debug.Log("Unimplemented");
            return child;
        }


    }
}
