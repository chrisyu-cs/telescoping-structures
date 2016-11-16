using UnityEngine;
using System.Collections.Generic;
using System;

namespace Telescopes
{
    /// <summary>
    /// A class that represents a telescoping shell.
    /// </summary>
    [RequireComponent(typeof(MeshFilter))]
    public class TelescopeShell : TelescopeElement
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

        public TelescopeSegment containingSegment;

        List<Vector3> bottomVerts;
        List<Vector3> topVerts;

        bool HasOverhang;

        public List<Vector3> BottomRing { get { return bottomVerts; } }
        public List<Vector3> TopRing { get { return topVerts; } }

        public Mesh mesh
        {
            get { return mFilter.mesh; }
        }

        public Bounds WorldSpaceBounds
        {
            get { return mRenderer.bounds; }
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

        Vector3 getLocalLocationAlongPath(float t, float curvature, float torsion)
        {
            float arcLength = t * length;
            return translationOfDistance(arcLength, curvature, torsion);
        }

        public Vector3 translationOfDistance(float arcLength)
        {
            return TelescopeUtils.TranslateAlongHelix(curvature, torsion, arcLength);
        }

        Vector3 translationOfDistance(float arcLength, float curvature, float torsion)
        {
            return TelescopeUtils.TranslateAlongHelix(curvature, torsion, arcLength);
        }

        public override void ExtendImmediate(float t)
        {
            // Nothing; individual shells don't extend.
        }

        public Quaternion rotationOfDistance(float arcLength, float curvature, float torsion)
        {
            return TelescopeUtils.RotateAlongHelix(curvature, torsion, arcLength);
        }

        public Quaternion getLocalRotationAlongPath(float t)
        {
            return getLocalRotationAlongPath(t, curvature, torsion);
        }

        Quaternion getLocalRotationAlongPath(float t, float curvature, float torsion)
        {
            if (curvature > 1e-6)
            {
                // Compute how many radians along the circle we moved.
                float arcLength = t * length;
                Quaternion rotation = rotationOfDistance(arcLength, curvature, torsion);
                return rotation;
            }
            else
            {
                return Quaternion.identity;
            }
        }

        public Vector3 getDirectionAlongPath(float t)
        {
            // TODO: compute as if moving along circle
            return getLocalRotationAlongPath(t) * Vector3.forward;
        }

        Vector3 getDirectionAlongPath(float t, float curvature, float torsion)
        {
            return getLocalRotationAlongPath(t, curvature, torsion) * Vector3.forward;
        }

        public Vector3 getNormalAlongPath(float t)
        {
            return getLocalRotationAlongPath(t) * Vector3.up;
        }

        public Vector3 getNormalAlongPath(float t, float curvature, float torsion)
        {
            return getLocalRotationAlongPath(t, curvature, torsion) * Vector3.up;
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

            topVerts = topOuter.ConvertAll<Vector3>(new Converter<IndexedVertex, Vector3>(IndexedVertex.toVector3));

            if (topOuter.Count != topInner.Count)
            {
                throw new System.Exception("Circles do not have same number of vertices.");
            }

            // Since the cuts are not necessarily aligned (in the case of curvature
            // or torsion changes), we want to stitch using the closest pairs of vertices.
            // Since we start with outer vertex i = 0, find the index of the inner vertex
            // closest to outer 0.
            Vector3 outer0top = topOuter[0].vertex;
            int topInnerOffset = 0;
            float closestDistance = Vector3.Distance(outer0top, topInner[0].vertex);

            for (int i = 0; i < topInner.Count; i++)
            {
                float dist = Vector3.Distance(topInner[i].vertex, outer0top);
                if (dist < closestDistance)
                {
                    closestDistance = dist;
                    topInnerOffset = i;
                }
            }

            // To join the top rim, we need to add the triangles with
            // counterclockwise ordering, since the vertices are clockwise
            // when viewed from below (not above).
            for (int outerI = 0; outerI < topOuter.Count; outerI++)
            {
                int outerIPlus = (outerI + 1) % topOuter.Count;
                int innerI = (outerI + topInnerOffset) % topInner.Count;
                int innerIPlus = (innerI + 1) % topInner.Count;

                // The triangles with two vertices in the outer circle have
                // the order outer[i], inner[i], outer[i+1].
                IndexedVertex iv1 = topOuter[outerI];
                IndexedVertex iv2 = topInner[innerI];
                IndexedVertex iv3 = topOuter[outerIPlus];
                IndexTriangle tri = new IndexTriangle(iv1, iv2, iv3);
                triangles.Add(tri);

                // The triangles with two vertices in the inner circle have
                // the order inner[i], inner[i+1], outer[i+1].
                iv1 = topInner[innerI];
                iv2 = topInner[innerIPlus];
                iv3 = topOuter[outerIPlus];
                tri = new IndexTriangle(iv1, iv2, iv3);
                triangles.Add(tri);
            }

            List<IndexedVertex> bottomOuter = outer.circleCuts[0];
            List<IndexedVertex> bottomInner = inner.circleCuts[0];

            bottomVerts = bottomOuter.ConvertAll<Vector3>(new Converter<IndexedVertex, Vector3>(IndexedVertex.toVector3));

            if (bottomOuter.Count != bottomInner.Count)
            {
                throw new System.Exception("Circles do not have same number of vertices.");
            }

            // Same procedure as top ring: find starting vertex closest to
            // vertex 0 of outer ring.
            Vector3 outer0bottom = bottomOuter[0].vertex;
            int bottomInnerOffset = 0;
            closestDistance = Vector3.Distance(outer0bottom, bottomInner[0].vertex);

            for (int i = 0; i < bottomInner.Count; i++)
            {
                float dist = Vector3.Distance(bottomInner[i].vertex, outer0bottom);
                if (dist < closestDistance)
                {
                    closestDistance = dist;
                    bottomInnerOffset = i;
                }
            }

            // To join the bottom faces, we add triangles with vertices in clockwise order.
            for (int outerI = 0; outerI < bottomOuter.Count; outerI++)
            {
                int outerIPlus = (outerI + 1) % bottomOuter.Count;
                int innerI = (outerI + bottomInnerOffset) % bottomInner.Count;
                int innerIPlus = (innerI + 1) % bottomInner.Count;

                // The triangles with two vertices in the outer circle have
                // the order outer[i], outer[i+1], inner[i].
                IndexedVertex iv1 = bottomOuter[outerI];
                IndexedVertex iv2 = bottomOuter[outerIPlus];
                IndexedVertex iv3 = bottomInner[innerI];
                IndexTriangle tri = new IndexTriangle(iv1, iv2, iv3);
                triangles.Add(tri);

                // The triangles with two vertices in the inner circle have
                // the order inner[i], outer[i+1], inner[i+1].
                iv1 = bottomInner[innerI];
                iv2 = bottomOuter[outerIPlus];
                iv3 = bottomInner[innerIPlus];
                tri = new IndexTriangle(iv1, iv2, iv3);
                triangles.Add(tri);
            }

            return triangles;
        }

        List<IndexTriangle> CloseCylinderCaps(CylinderMesh cylinder, List<IndexedVertex> verts)
        {
            List<IndexTriangle> triangles = new List<IndexTriangle>();

            List<IndexedVertex> bottomCircle = cylinder.circleCuts[0];
            List<IndexedVertex> topCircle = cylinder.circleCuts[cylinder.circleCuts.Count - 1];

            // Compute center point of bottom circle.
            Vector3 bottomCenter = Vector3.zero;
            foreach (IndexedVertex v in bottomCircle)
            {
                bottomCenter += v.vertex;
            }
            bottomCenter /= bottomCircle.Count;

            // Compute center point of top circle.
            Vector3 topCenter = Vector3.zero;
            foreach (IndexedVertex v in topCircle)
            {
                topCenter += v.vertex;
            }
            topCenter /= topCircle.Count;

            // Add the two points to the mesh.
            IndexedVertex bottomCenterVert = new IndexedVertex(bottomCenter, currentIndex);
            currentIndex++;
            IndexedVertex topCenterVert = new IndexedVertex(topCenter, currentIndex);
            currentIndex++;

            verts.Add(bottomCenterVert);
            verts.Add(topCenterVert);

            // Add triangle fans.
            for (int i = 0; i < bottomCircle.Count; i++)
            {
                // Bottom circle gets CCW order
                int iPlus = (i + 1) % bottomCircle.Count;

                IndexedVertex i1 = bottomCenterVert;
                IndexedVertex i2 = bottomCircle[i];
                IndexedVertex i3 = bottomCircle[iPlus];

                IndexTriangle tri = new IndexTriangle(i1, i2, i3);
                triangles.Add(tri);
            }

            for (int i = 0; i < topCircle.Count; i++)
            {
                // Top circle gets CW order
                int iPlus = (i + 1) % topCircle.Count;

                IndexedVertex i1 = topCenterVert;
                IndexedVertex i2 = topCircle[iPlus];
                IndexedVertex i3 = topCircle[i];

                IndexTriangle tri = new IndexTriangle(i1, i2, i3);
                triangles.Add(tri);
            }
            
            return triangles;
        }

        CylinderMesh GenerateCylinder(TelescopeParameters tParams, float slope,
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
                radiusLoss = i * lengthStep * slope * tParams.length;
                Vector3 centerPoint = getLocalLocationAlongPath(i * lengthStep, tParams.curvature, tParams.torsion);
                Vector3 facingDirection = getDirectionAlongPath(i * lengthStep, tParams.curvature, tParams.torsion);
                Vector3 normalDirection = getNormalAlongPath(i * lengthStep, tParams.curvature, tParams.torsion);

                bool doInner = (innerGroove && i < Constants.CUTS_PER_CYLINDER + Constants.FIN_CUTS);
                bool doOuter = (outerGroove && i <= Constants.FIN_CUTS);

                List<IndexedVertex> circle = GenerateCircle(i, centerPoint, facingDirection, normalDirection,
                    tParams.radius - radiusLoss, addInnerGrooves: doInner, addOuterGroove: doOuter);
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

        CylinderMesh GenerateInnerCylinder(TelescopeParameters nextParams, bool overhang, float arcOffset)
        {
            TelescopeParameters ourParams = getParameters();
            CylinderMesh innerCyl;

            // If there is a change in torsion from this shell to the next...
            if (nextParams != null && (nextParams.curvature != curvature || nextParams.torsion != torsion))
            {
                // Generate the outer profile of the child shell; this will become
                // the inner profile of this shell.
                TelescopeParameters innerParams = new TelescopeParameters(nextParams.length, nextParams.radius,
                    nextParams.thickness, nextParams.curvature, nextParams.torsion, nextParams.twistFromParent);
                innerCyl = GenerateCylinder(innerParams,
                    Constants.TAPER_SLOPE, innerGroove: true, overhang: overhang);

                // Move and rotate the inner profile so that the end circles align.
                Quaternion alignRotation = TelescopeUtils.childBaseRotation(ourParams, nextParams);
                Vector3 alignTranslation = TelescopeUtils.childBasePosition(ourParams, nextParams);

                Quaternion backRotation = TelescopeUtils.RotateAlongHelix(nextParams.curvature,
                    nextParams.torsion, arcOffset);
                Vector3 backTranslation = TelescopeUtils.TranslateAlongHelix(nextParams.curvature,
                    nextParams.torsion, arcOffset);

                innerCyl.ApplyRotation(backRotation);
                innerCyl.ApplyTranslation(backTranslation);

                innerCyl.ApplyRotation(alignRotation);
                innerCyl.ApplyTranslation(alignTranslation);

                Debug.Log("Outer radius = " + radius + ", inner radius = " + nextParams.radius);
            }

            else
            {
                TelescopeParameters innerParams = new TelescopeParameters(length, radius - thickness,
                    thickness, curvature, torsion, 0);

                innerCyl = GenerateCylinder(innerParams,
                    Constants.TAPER_SLOPE, innerGroove: true, overhang: overhang);

                if (nextParams != null)
                {
                    Quaternion backRotation = TelescopeUtils.RotateAlongHelix(nextParams.curvature,
                        nextParams.torsion, arcOffset);
                    Vector3 backTranslation = TelescopeUtils.TranslateAlongHelix(nextParams.curvature,
                        nextParams.torsion, arcOffset);

                    innerCyl.ApplyRotation(backRotation);
                    innerCyl.ApplyTranslation(backTranslation);
                }
            }

            return innerCyl;
        }

        public Mesh GenerateInnerVolume(TelescopeParameters nextParams, float arcOffset)
        {
            currentIndex = 0;

            Mesh mesh = new Mesh();

            // Get the inner cylinder of the shell
            CylinderMesh inner = GenerateInnerCylinder(nextParams, HasOverhang, arcOffset);

            // Get the vertices
            List<IndexedVertex> verticesList = flattenList(inner.circleCuts);
            // Create the triangles for the two ends of the cylinder; this also
            // adds 2 vertices to the list.
            List<IndexTriangle> capTris = CloseCylinderCaps(inner, verticesList);
            // Get the positions out of the list
            List<Vector3> vertices = verticesList.ConvertAll(new
                System.Converter<IndexedVertex, Vector3>(IndexedVertex.toVector3));
            List<int> indicesList = new List<int>();
            
            // Add triangles for the cylinder itself
            foreach (IndexTriangle tri in inner.triangles)
            {
                tri.addFrontFace(indicesList);
            }

            // Add triangles for the two ends
            foreach (IndexTriangle tri in capTris)
            {
                tri.addFrontFace(indicesList);
            }

            mesh.vertices = vertices.ToArray();
            mesh.triangles = indicesList.ToArray();
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();

            return mesh;
        }

        public void GenerateGeometry(TelescopeParameters theParams, TelescopeParameters nextParams,
            bool overhang = true, bool outerGroove = true)
        {
            HasOverhang = overhang;

            this.thickness = theParams.thickness;
            this.length = theParams.length;
            this.radius = theParams.radius;
            this.curvature = theParams.curvature;
            this.torsion = theParams.torsion;
            this.twistAngle = theParams.twistFromParent;
            this.nextTwistAngle = (nextParams != null) ? nextParams.twistFromParent : 0;

            // Reset the mesh.
            currentIndex = 0;
            mFilter.mesh.Clear();

            float slopeCosmetic = thickness * Constants.COSMETIC_TAPER_RATIO;

            TelescopeParameters outerParams = new TelescopeParameters(length, radius - Constants.SHELL_GAP,
                thickness, curvature, torsion, 0);

            CylinderMesh outerCyl = GenerateCylinder(outerParams,
                slopeCosmetic + Constants.TAPER_SLOPE,
                outerGroove: outerGroove, overhang: overhang);

            CylinderMesh innerCyl = GenerateInnerCylinder(nextParams, overhang, 0);

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
        }

        public void ReplaceMesh(Mesh m)
        {
            mFilter.mesh = m;
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
                TelescopeShell parent = transform.parent.GetComponent<TelescopeShell>();
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

        public void ApplyRotation(Quaternion rotation)
        {
            foreach (List<IndexedVertex> circle in circleCuts)
            {
                foreach (IndexedVertex vert in circle)
                {
                    vert.vertex = rotation * vert.vertex;
                }
            }
        }

        public void ApplyTranslation(Vector3 translation)
        {
            foreach (List<IndexedVertex> circle in circleCuts)
            {
                foreach (IndexedVertex vert in circle)
                {
                    vert.vertex = vert.vertex + translation;
                }
            }
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