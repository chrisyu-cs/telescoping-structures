using UnityEngine;
using System.Collections.Generic;
using System;

namespace Telescopes
{
    /// <summary>
    /// A class that represents a telescoping shell.
    /// </summary>
    [RequireComponent(typeof(MeshFilter))]
    public class TelescopingShell : TelescopeElement
    {
        private MeshFilter mFilter;
        private MeshRenderer mRenderer;
        private int currentIndex;

        public float length, radius, thickness;
        public float curvature, torsion;
        public float twistAngle;
        public float nextTwistAngle;

        public bool Reversed;

        public Vector3 baseTranslation;
        public Quaternion baseRotation;
        // public float baseRadians;

        public bool isRoot = false;
        public float extensionRatio = 0;

        private float oldRatio = 0;
        private float interpTimespan = 0;
        private bool currentlyInterp = false;
        private float targetRatio = 0;
        private float ratioInterpTime = 0;

        public TelescopingSegment containingSegment;

        public Mesh mesh
        {
            get { return mFilter.mesh; }
        }

        // Use this for initialization
        void Awake()
        {
            currentIndex = 0;
            mFilter = GetComponent<MeshFilter>();
            mRenderer = GetComponent<MeshRenderer>();
            if (!mFilter)
            {
                mFilter = gameObject.AddComponent<MeshFilter>();
            }
            if (!mRenderer)
            {
                mRenderer = gameObject.AddComponent<MeshRenderer>();
            }
        }

        public bool IsTerminal()
        {
            return (transform.childCount == 0);
        }

        public override int numChildElements()
        {
            return 1;
        }

        public override TelescopeElement getChildElement(int i)
        {
            return this;
        }

        public TelescopeParameters getParameters()
        {
            TelescopeParameters tp = new TelescopeParameters(length, radius, thickness, curvature, torsion, twistAngle);
            return tp;
        }

        public void setMaterial(Material m)
        {
            mRenderer.material = m;
        }

        public float radiansOfLength(float arcLength)
        {
            if (curvature > 1e-6)
                return arcLength * curvature;
            else return arcLength;
        }

        public override Vector3 getAttachmentLocation(float t)
        {
            if (!Reversed) return getLocalLocationAlongPath(t);
            else return getLocalLocationAlongPath(1 - t);
        }

        public override Quaternion getAttachmentRotation(float t)
        {
            if (!Reversed) return getLocalRotationAlongPath(t);
            else return getLocalRotationAlongPath(1 - t);
        }

        public Vector3 getInvLocationAlongPath(float t)
        {
            return getLocalLocationAlongPath(-t);
        }

        public Vector3 getLocalLocationAlongPath(float t)
        {
            float arcLength = t * length;
            return translationOfDistance(arcLength);
        }

        public Vector3 translationOfDistance(float arcLength)
        {
            return TelescopeUtils.TranslateAlongHelix(curvature, torsion, arcLength);
        }

        public override void ExtendImmediate(float t)
        {
            // Nothing; individual shells don't extend.
        }

        public Quaternion getLocalRotationAlongPath(float t)
        {
            if (curvature > 1e-6)
            {
                // Compute how many radians along the circle we moved.
                float arcLength = t * length;
                Quaternion rotation = rotationOfDistance(arcLength);
                return rotation;
            }
            else
            {
                return Quaternion.identity;
            }
        }

        public Quaternion rotationOfDistance(float arcLength)
        {
            return TelescopeUtils.RotateAlongHelix(curvature, torsion, arcLength);
        }

        public Vector3 getDirectionAlongPath(float t)
        {
            // TODO: compute as if moving along circle
            return getLocalRotationAlongPath(t) * Vector3.forward;
        }

        public Vector3 getNormalAlongPath(float t)
        {
            return getLocalRotationAlongPath(t) * Vector3.up;
        }

        bool IsInRadius(int x, int bound1, int bound2, int radius)
        {
            int upperBound = Mathf.Max(bound1, bound2);
            int lowerBound = Mathf.Min(bound1, bound2);

            return (x > lowerBound - radius && x < upperBound + radius);
        }

        #region GenGeometry
        /// <summary>
        /// Create a set of vertices arranged evenly spaced in a circle, with the specified radius,
        /// centered at centerPoint, and with the specified normal direction. The number of vertices
        /// is given by verticesPerCircle.
        /// </summary>
        /// <param name="centerPoint"></param>
        /// <param name="direction"></param>
        /// <param name="radius"></param>
        /// <returns></returns>
        List<IndexedVertex> GenerateCircle(int circNum, Vector3 centerPoint, Vector3 direction,
            Vector3 normal, float radius, bool addInnerGrooves = false, bool addOuterGroove = false)
        {
            float angleStep = (2 * Mathf.PI) / Constants.VERTS_PER_CIRCLE;
            float degreeStep = angleStep * Mathf.Rad2Deg;

            int twistCuts = Mathf.CeilToInt(Mathf.Abs(nextTwistAngle) / degreeStep);
            twistCuts *= Mathf.RoundToInt(Mathf.Sign(nextTwistAngle));

            List<IndexedVertex> verts = new List<IndexedVertex>();
            
            int grooveRange = Constants.GROOVE_CUT_RADIUS;

            // First create points in a circle in the XY plane, facing the forward direction.
            // Then apply the rotation that will rotate the normal onto the desired direction.
            // Finally, offset it in space to the desired location.
            Quaternion circleRotation = Quaternion.FromToRotation(Vector3.forward, direction);
            Vector3 initNormal = Vector3.up;
            Vector3 rotatedNormal = circleRotation * initNormal;

            float angle = TelescopeUtils.AngleBetween(rotatedNormal, normal, direction);
            Quaternion normalRotation = Quaternion.AngleAxis(angle, direction);

            for (int i = 0; i < Constants.VERTS_PER_CIRCLE; i++)
            {
                float currentAngle = i * angleStep;
                float radiusOffset = 0;
                if (i < grooveRange ||
                    Mathf.Abs(i - Constants.VERTS_PER_CIRCLE / 2) < grooveRange ||
                    Mathf.Abs(i - Constants.VERTS_PER_CIRCLE) < grooveRange)
                {
                    if (addInnerGrooves)
                    {
                        radiusOffset = (thickness - Constants.SHELL_GAP) * Constants.INDENT_RATIO;
                    }
                    else if (addOuterGroove)
                    {
                        radiusOffset = (thickness - Constants.SHELL_GAP / 2) * Constants.INDENT_RATIO;
                    }
                }
                
                else if (circNum >= Constants.CUTS_PER_CYLINDER - 1
                    && circNum < Constants.CUTS_PER_CYLINDER + Constants.FIN_CUTS)
                {
                    if (addInnerGrooves &&
                        (IsInRadius(i, 0, twistCuts, grooveRange) ||
                        IsInRadius(i, Constants.VERTS_PER_CIRCLE,
                            Constants.VERTS_PER_CIRCLE + twistCuts, grooveRange) ||
                        IsInRadius(i, Constants.VERTS_PER_CIRCLE / 2,
                            Constants.VERTS_PER_CIRCLE / 2 + twistCuts, grooveRange)))
                    {
                        radiusOffset = (thickness - Constants.SHELL_GAP) * Constants.INDENT_RATIO;
                    }
                }

                // Make the vertices in clockwise order
                Vector3 vert = new Vector3(Mathf.Cos(currentAngle), -Mathf.Sin(currentAngle));
                // Scale by radius.
                vert *= (radius + radiusOffset);
                // Rotate it to orbit the desired direction.
                vert = circleRotation * vert;
                // Rotate it again so that the curvature normal is aligned.
                vert = normalRotation * vert;
                // Offset in space to the center point.
                vert += centerPoint;
                IndexedVertex iv = new IndexedVertex(vert, currentIndex);
                currentIndex++;
                verts.Add(iv);
            }
            return verts;
        }

        List<IndexTriangle> StitchCircles(List<IndexedVertex> circle1, List<IndexedVertex> circle2)
        {
            // Create triangles that connect circle1 to circle2, forming a band of a surface.
            if (circle1.Count != circle2.Count)
            {
                throw new System.Exception("Circles do not have the same number of vertices.");
            }

            List<IndexTriangle> triangles = new List<IndexTriangle>();

            for (int i = 0; i < circle1.Count; i++)
            {
                // Create the triangles that have 2 points each on circle1.
                // These all have the ordering (c2[i], c1[i+1], c1[i]).
                int iPlus = (i + 1) % circle1.Count;
                IndexedVertex iv1 = circle2[i];
                IndexedVertex iv2 = circle1[iPlus];
                IndexedVertex iv3 = circle1[i];
                IndexTriangle tri = new IndexTriangle(iv1, iv2, iv3);
                triangles.Add(tri);

                // Create the triangles that have 2 points each on circle2.
                // These have the ordering (c2[i], c2[i+1], c1[i+1].
                iv1 = circle2[i];
                iv2 = circle2[iPlus];
                iv3 = circle1[iPlus];
                tri = new IndexTriangle(iv1, iv2, iv3);
                triangles.Add(tri);
            }
            return triangles;
        }

        List<IndexedVertex> flattenList(List<List<IndexedVertex>> lst)
        {
            // Flatten vertex list
            List<IndexedVertex> flattenedVerts = new List<IndexedVertex>();
            foreach (List<IndexedVertex> verts in lst)
            {
                flattenedVerts.AddRange(verts);
            }
            return flattenedVerts;
        }

        List<IndexTriangle> stitchCylinderEnds(CylinderMesh outer, CylinderMesh inner)
        {
            if (outer.circleCuts.Count != inner.circleCuts.Count)
            {
                throw new System.Exception("Cylinders do not have same number of cuts.");
            }
            int numCuts = outer.circleCuts.Count;
            List<IndexTriangle> triangles = new List<IndexTriangle>();

            List<IndexedVertex> topOuter = outer.circleCuts[numCuts - 1];
            List<IndexedVertex> topInner = inner.circleCuts[numCuts - 1];

            if (topOuter.Count != topInner.Count)
            {
                throw new System.Exception("Circles do not have same number of vertices.");
            }

            // To join the top rim, we need to add the triangles with
            // counterclockwise ordering, since the vertices are clockwise
            // when viewed from below (not above).
            for (int i = 0; i < topOuter.Count; i++)
            {
                int iPlus = (i + 1) % topOuter.Count;
                // The triangles with two vertices in the outer circle have
                // the order outer[i], inner[i], outer[i+1].
                IndexedVertex iv1 = topOuter[i];
                IndexedVertex iv2 = topInner[i];
                IndexedVertex iv3 = topOuter[iPlus];
                IndexTriangle tri = new IndexTriangle(iv1, iv2, iv3);
                triangles.Add(tri);

                // The triangles with two vertices in the inner circle have
                // the order inner[i], inner[i+1], outer[i+1].
                iv1 = topInner[i];
                iv2 = topInner[iPlus];
                iv3 = topOuter[iPlus];
                tri = new IndexTriangle(iv1, iv2, iv3);
                triangles.Add(tri);
            }

            List<IndexedVertex> bottomOuter = outer.circleCuts[0];
            List<IndexedVertex> bottomInner = inner.circleCuts[0];

            if (bottomOuter.Count != bottomInner.Count)
            {
                throw new System.Exception("Circles do not have same number of vertices.");
            }

            // To join the bottom faces, we add triangles with vertices in clockwise order.
            for (int i = 0; i < bottomOuter.Count; i++)
            {
                int iPlus = (i + 1) % bottomOuter.Count;
                // The triangles with two vertices in the outer circle have
                // the order outer[i], outer[i+1], inner[i].
                IndexedVertex iv1 = bottomOuter[i];
                IndexedVertex iv2 = bottomOuter[iPlus];
                IndexedVertex iv3 = bottomInner[i];
                IndexTriangle tri = new IndexTriangle(iv1, iv2, iv3);
                triangles.Add(tri);

                // The triangles with two vertices in the inner circle have
                // the order inner[i], outer[i+1], inner[i+1].
                iv1 = bottomInner[i];
                iv2 = bottomOuter[iPlus];
                iv3 = bottomInner[iPlus];
                tri = new IndexTriangle(iv1, iv2, iv3);
                triangles.Add(tri);
            }

            return triangles;
        }

        CylinderMesh GenerateCylinder(float length, float radius, float curvatureAmount, float slope,
            bool innerGroove = false, bool outerGroove = false, bool overhang = true)
        {
            // We basically need to sweep a circular cross-section along a circular path.
            List<List<IndexedVertex>> circles = new List<List<IndexedVertex>>();

            float lengthStep = 1f / (Constants.CUTS_PER_CYLINDER - 1);

            float radiusLoss = 0;

            int numCircles = Constants.CUTS_PER_CYLINDER + (overhang ? Constants.OVERHANG_CUTS : 0);

            // Generate vertices
            for (int i = 0; i < numCircles; i++)
            {
                radiusLoss = i * lengthStep * slope * length;
                Vector3 centerPoint = getLocalLocationAlongPath(i * lengthStep);
                Vector3 facingDirection = getDirectionAlongPath(i * lengthStep);
                Vector3 normalDirection = getNormalAlongPath(i * lengthStep);

                bool doInner = (innerGroove && i < Constants.CUTS_PER_CYLINDER + Constants.FIN_CUTS);
                bool doOuter = (outerGroove && i <= Constants.FIN_CUTS);

                List<IndexedVertex> circle = GenerateCircle(i, centerPoint, facingDirection, normalDirection,
                    radius - radiusLoss, addInnerGrooves: doInner, addOuterGroove: doOuter);
                circles.Add(circle);
            }

            // Now generate faces
            List<IndexTriangle> allIndices = new List<IndexTriangle>();
            for (int i = 0; i < numCircles - 1; i++)
            {
                List<IndexTriangle> tris = StitchCircles(circles[i], circles[i + 1]);
                allIndices.AddRange(tris);
            }

            CylinderMesh cm = new CylinderMesh(circles, allIndices);

            return cm;
        }

        public void GenerateGeometry(TelescopeParameters theParams, float nextTwist, bool overhang = true)
        {
            this.thickness = theParams.thickness;
            this.length = theParams.length;
            this.radius = theParams.radius;
            this.curvature = theParams.curvature;
            this.torsion = theParams.torsion;
            this.twistAngle = theParams.twistFromParent;
            this.nextTwistAngle = nextTwist;

            // Reset the mesh.
            currentIndex = 0;
            mFilter.mesh.Clear();

            float slopeCosmetic = thickness * Constants.COSMETIC_TAPER_RATIO;

            CylinderMesh outerCyl = GenerateCylinder(length, radius - Constants.SHELL_GAP, curvature,
                slopeCosmetic + Constants.TAPER_SLOPE, outerGroove: true, overhang: overhang);
            CylinderMesh innerCyl = GenerateCylinder(length, radius - thickness, curvature,
                Constants.TAPER_SLOPE, innerGroove: true, overhang: overhang);

            // Flatten vertex list
            List<IndexedVertex> outerVerts = flattenList(outerCyl.circleCuts);
            List<IndexedVertex> innerVerts = flattenList(innerCyl.circleCuts);
            outerVerts.AddRange(innerVerts);

            // Get the triangles that join the inner and outer surfaces.
            List<IndexTriangle> edgeTriangles = stitchCylinderEnds(outerCyl, innerCyl);

            // Sort the vertices just to be safe
            outerVerts.Sort();

            // Make vertex buffer
            List<Vector3> vecs = outerVerts.ConvertAll(new System.Converter<IndexedVertex, Vector3>(IndexedVertex.toVector3));
            Vector3[] vertices = vecs.ToArray();

            // Make index buffer
            List<int> indicesList = new List<int>();

            foreach (IndexTriangle tri in outerCyl.triangles)
            {
                tri.addFrontFace(indicesList);
            }
            foreach (IndexTriangle tri in innerCyl.triangles)
            {
                tri.addBackFace(indicesList);
            }
            foreach (IndexTriangle tri in edgeTriangles)
            {
                tri.addFrontFace(indicesList);
            }

            mFilter.mesh.vertices = vertices;
            mFilter.mesh.triangles = indicesList.ToArray();

            mFilter.mesh.RecalculateBounds();
            mFilter.mesh.RecalculateNormals();

            //AddFins();
        }

        public float getTaperLoss()
        {
            return length * Constants.TAPER_SLOPE;
        }
        #endregion

        public void extendToRatio(float targetT, float overTime)
        {
            interpTimespan = overTime;
            oldRatio = extensionRatio;
            targetRatio = targetT;
            ratioInterpTime = 0;
            currentlyInterp = true;
        }

        public void SetTransform()
        {
            float extendT = Mathf.Clamp01(extensionRatio * 2);
            float twistT = Mathf.Clamp01(extensionRatio * 2 - 1);

            if (!Reversed)
            {
                Vector3 localTranslation = getLocalLocationAlongPath(extendT);
                Quaternion localRotation = getLocalRotationAlongPath(extendT);
                
                // Add twist angle.
                Vector3 forwardAxis = localRotation * baseRotation * Vector3.forward;
                Quaternion roll = Quaternion.AngleAxis(-twistAngle * twistT, forwardAxis);
                localRotation = roll * localRotation;

                // Set the shell's local translation from parent based on how extended it is.
                transform.localPosition = baseRotation * localTranslation + baseTranslation;
                transform.localRotation = baseRotation * localRotation;
            }

            else
            {
                TelescopingShell parent = transform.parent.GetComponent<TelescopingShell>();
                if (!parent) return;

                Vector3 localTranslation = parent.getInvLocationAlongPath(extendT);
                Quaternion localRotation = parent.getLocalRotationAlongPath(extendT);

                Vector3 forwardPosition = Quaternion.Inverse(baseRotation) * localTranslation - parent.baseTranslation;
                Quaternion forwardRotation = Quaternion.Inverse(parent.baseRotation) * Quaternion.Inverse(localRotation);

                // Set the shell's local translation from parent based on how extended it is.
                transform.localRotation = forwardRotation;
                transform.localPosition = forwardPosition;

                // Now we need to rotate by the twist angle, with the pivot being the parent's angle.
                Vector3 parentWorld = parent.transform.position;
                Vector3 axis = parent.transform.forward;
                transform.RotateAround(parentWorld, axis, parent.twistAngle * twistT);
            }
        }

        public void WriteShell()
        {
            string name = this.name + ".stl";
            STLWriter.WriteSTLOfMesh(mFilter.mesh, name);
        }
    }

    class CylinderMesh
    {
        public List<List<IndexedVertex>> circleCuts;
        public List<IndexTriangle> triangles;

        public CylinderMesh(List<List<IndexedVertex>> vs, List<IndexTriangle> ts)
        {
            circleCuts = vs;
            triangles = ts;
        }
    }

    class IndexedVertex : System.IComparable<IndexedVertex>
    {
        public Vector3 vertex;
        public int index;

        public static Vector3 toVector3(IndexedVertex iv)
        {
            return iv.vertex;
        }

        public IndexedVertex(Vector3 v, int i)
        {
            vertex = v;
            index = i;
        }

        public int CompareTo(IndexedVertex other)
        {
            return index.CompareTo(other.index);
        }
    }

    class IndexTriangle
    {
        public int i1, i2, i3;

        public IndexTriangle(IndexedVertex iv1, IndexedVertex iv2, IndexedVertex iv3)
        {
            i1 = iv1.index;
            i2 = iv2.index;
            i3 = iv3.index;
        }

        public void addFrontFace(List<int> accumulator)
        {
            accumulator.Add(i1);
            accumulator.Add(i2);
            accumulator.Add(i3);
        }

        public void addBackFace(List<int> accumulator)
        {
            accumulator.Add(i1);
            accumulator.Add(i3);
            accumulator.Add(i2);
        }
    }
}