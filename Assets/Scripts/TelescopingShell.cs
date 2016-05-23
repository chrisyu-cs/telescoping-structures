using UnityEngine;
using System.Collections.Generic;

namespace Telescopes
{
    /// <summary>
    /// A class that represents a telescoping shell.
    /// </summary>
    [RequireComponent(typeof(MeshFilter))]
    public class TelescopingShell : MonoBehaviour
    {
        private MeshFilter mFilter;
        private MeshRenderer mRenderer;
        private int currentIndex;

        public float length, radius, thickness;
        public float curvatureAmount;
        public float twistAngle;

        public Vector3 baseTranslation;
        public Quaternion baseRotation;
        // public float baseRadians;

        public bool isRoot = false;
        public float extensionRatio = 1;

        private float oldRatio = 0;
        private float interpTimespan = 0;
        private bool currentlyInterp = false;
        private float targetRatio = 0;
        private float ratioInterpTime = 0;

        public TelescopingSegment containingSegment;

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

        public TelescopeParameters getParameters()
        {
            TelescopeParameters tp = new TelescopeParameters(length, radius, thickness, curvatureAmount, twistAngle);
            return tp;
        }

        public void setMaterial(Material m)
        {
            mRenderer.material = m;
        }

        public float radiansOfLength(float arcLength)
        {
            if (curvatureAmount > 1e-6)
                return arcLength * curvatureAmount;
            else return arcLength;
        }

        public Vector3 getLocalLocationAlongPath(float t)
        {
            float arcLength = t * length;
            return translationOfDistance(arcLength);
        }

        public Vector3 translationOfDistance(float arcLength)
        {
            return TelescopeUtils.translateAlongCircle(curvatureAmount, arcLength);
        }

        public Quaternion getLocalRotationAlongPath(float t, float twistT = 0)
        {
            if (curvatureAmount > 1e-6)
            {
                // Compute how many radians along the circle we moved.
                float arcLength = t * length;
                Quaternion rotation = rotationOfDistance(arcLength);
                if (twistT > 0)
                {
                    Vector3 forwardAxis = rotation * baseRotation * Vector3.forward;
                    Quaternion roll = Quaternion.AngleAxis(-twistAngle * twistT, forwardAxis);
                    return roll * rotation;
                }
                return rotation;
            }
            else
            {
                Vector3 forwardAxis = baseRotation * Vector3.forward;
                Quaternion roll = Quaternion.AngleAxis(-twistAngle * twistT, forwardAxis);
                return roll;
            }
        }

        public Quaternion rotationOfDistance(float arcLength)
        {
            return TelescopeUtils.rotateAlongCircle(curvatureAmount, arcLength);
        }

        public Vector3 getDirectionAlongPath(float t)
        {
            // TODO: compute as if moving along circle
            return getLocalRotationAlongPath(t) * Vector3.forward;
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
        List<IndexedVertex> GenerateCircle(int circNum, Vector3 centerPoint, Vector3 direction, float radius)
        {
            float angleStep = (2 * Mathf.PI) / TelescopingSegment.verticesPerCircle;
            List<IndexedVertex> verts = new List<IndexedVertex>();

            float uvY = (float)circNum / TelescopingSegment.cutsPerCylinder;

            // First create points in a circle in the XY plane, facing the forward direction.
            // Then apply the rotation that will rotate the normal onto the desired direction.
            // Finally, offset it in space to the desired location.
            Quaternion circleRotation = Quaternion.FromToRotation(Vector3.forward, direction);
            for (int i = 0; i < TelescopingSegment.verticesPerCircle; i++)
            {
                float uvX = i / TelescopingSegment.verticesPerCircle;
                // Make the vertices in clockwise order
                Vector3 vert = new Vector3(Mathf.Cos(i * angleStep), -Mathf.Sin(i * angleStep));
                // Scale by radius.
                vert *= radius;
                // Rotate it to orbit the desired direction.
                vert = circleRotation * vert;
                // Offset in space to the center point.
                vert += centerPoint;
                IndexedVertex iv = new IndexedVertex(vert, currentIndex, new Vector2(uvX, uvY));
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

        public void GenerateGeometry(TelescopeParameters theParams)
        {
            this.thickness = theParams.thickness;
            this.length = theParams.length;
            this.radius = theParams.radius;
            this.curvatureAmount = theParams.curvature;
            this.twistAngle = theParams.twistFromParent;

            // Reset the mesh.
            currentIndex = 0;
            mFilter.mesh.Clear();

            CylinderMesh outerCyl = GenerateCylinder(length, radius, curvatureAmount);
            CylinderMesh innerCyl = GenerateCylinder(length, radius - thickness, curvatureAmount);

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

            // Make UV buffer
            List<Vector2> uvList = outerVerts.ConvertAll(new System.Converter<IndexedVertex, Vector2>(IndexedVertex.toUV));
            Vector2[] uvs = uvList.ToArray();

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
            mFilter.mesh.uv = uvs;

            mFilter.mesh.RecalculateBounds();
            mFilter.mesh.RecalculateNormals();
        }

        CylinderMesh GenerateCylinder(float length, float radius, float curvatureAmount)
        {
            // We basically need to sweep a circular cross-section along a circular path.
            List<List<IndexedVertex>> circles = new List<List<IndexedVertex>>();

            float lengthStep = 1f / (TelescopingSegment.cutsPerCylinder - 1);

            // TODO: use curvatures

            // Generate vertices
            for (int i = 0; i < TelescopingSegment.cutsPerCylinder; i++)
            {
                Vector3 centerPoint = getLocalLocationAlongPath(i * lengthStep);
                Vector3 facingDirection = getDirectionAlongPath(i * lengthStep);
                List<IndexedVertex> circle = GenerateCircle(i, centerPoint, facingDirection, radius);
                circles.Add(circle);
                // TODO: update radius?
            }

            // Now generate faces
            List<IndexTriangle> allIndices = new List<IndexTriangle>();
            for (int i = 0; i < TelescopingSegment.cutsPerCylinder - 1; i++)
            {
                List<IndexTriangle> tris = StitchCircles(circles[i], circles[i + 1]);
                allIndices.AddRange(tris);
            }

            CylinderMesh cm = new CylinderMesh(circles, allIndices);

            return cm;
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

        // Update is called once per frame
        void Update()
        {
            if (isRoot)
            {
                return;
            }

            extensionRatio = Mathf.Clamp01(extensionRatio);

            if (currentlyInterp)
            {
                ratioInterpTime += Time.deltaTime;
                extensionRatio = Mathf.SmoothStep(oldRatio, targetRatio, ratioInterpTime / interpTimespan);
                if (ratioInterpTime > interpTimespan)
                {
                    extensionRatio = targetRatio;
                    currentlyInterp = false;
                }
            }

            Vector3 localTranslation = getLocalLocationAlongPath(Mathf.Clamp01(extensionRatio * 2));
            Quaternion localRotation = getLocalRotationAlongPath(Mathf.Clamp01(extensionRatio * 2),
                Mathf.Clamp01(extensionRatio * 2 - 1));

            // Set the shell's local translation from parent based on how extended it is.
            transform.localPosition = baseRotation * localTranslation + baseTranslation;
            transform.localRotation = localRotation * baseRotation;
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
        public Vector2 uv;
        public Vector3 vertex;
        public int index;

        public static Vector3 toVector3(IndexedVertex iv)
        {
            return iv.vertex;
        }

        public static Vector2 toUV(IndexedVertex iv)
        {
            return iv.uv;
        }

        public IndexedVertex(Vector3 v, int i, Vector2 u)
        {
            vertex = v;
            index = i;
            uv = u;
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