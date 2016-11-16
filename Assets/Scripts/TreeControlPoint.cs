using UnityEngine;
using System.Collections.Generic;

namespace Telescopes
{
    public class TreeControlPoint : MonoBehaviour
    {
        public SphereNode node;
        public InputTree containingTree;
        public TreeControlPoint parent;
        public List<TreeControlPoint> children;

        private Vector3 curvatureCenter;
        private static int nodeNum = 0;

        public bool hasCurvature;

        private GameObject sphere;
        private GameObject controlSphere;
        private SphereCollider sphereCollider;
        private LineRenderer lineRenderer;
        private MeshRenderer meshRenderer;

        private CurvatureControlPoint curvatureControlPt;

        public Vector3 curvaturePoint()
        {
            if (curvatureControlPt) return curvatureControlPt.transform.position;
            else return Vector3.zero;
        }

        void Start()
        {
            if (children == null) children = new List<TreeControlPoint>();
            if (node.radius == 0) node = new SphereNode(Constants.INITIAL_SPLINE_SIZE, transform.position);
            sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.transform.parent = transform;
            sphere.transform.localPosition = Vector3.zero;

            meshRenderer = sphere.GetComponent<MeshRenderer>();

            SphereCollider c = sphere.GetComponent<SphereCollider>();
            Destroy(c);
            sphereCollider = gameObject.AddComponent<SphereCollider>();
            sphereCollider.center = Vector3.zero;

            if (parent) curvatureCenter = (node.position + parent.node.position) / 2;

            Move(node.position);
            Resize(node.radius);
            gameObject.layer = 9;
        }

        public void SetupLine()
        {
            if (!parent) return;
            lineRenderer = gameObject.AddComponent<LineRenderer>();
            lineRenderer.material = containingTree.lineMaterial;

            Vector3[] positions = { parent.node.position, node.position };
            lineRenderer.SetPositions(positions);
        }

        public void Move(Vector3 pos)
        {
            node.position = pos;
            this.transform.position = pos;
        }

        public void Resize(float radius)
        {
            node.radius = radius;
            sphere.transform.localScale = new Vector3(2 * node.radius, 2 * node.radius, 2 * node.radius);
            sphereCollider.radius = radius;
        }

        public void ResizeDiff(float diff)
        {
            float oldRadius = node.radius;
            Resize(oldRadius + diff);
        }

        public TreeControlPoint AddNewChild()
        {
            GameObject childObj = new GameObject();
            childObj.name = "node" + nodeNum;
            nodeNum++;

            TreeControlPoint childPt = childObj.AddComponent<TreeControlPoint>();
            childPt.node.radius = node.radius;
            childPt.node.position = node.position + 0.2f * Vector3.up;
            this.children.Add(childPt);
            childPt.parent = this;
            childPt.transform.parent = containingTree.transform;
            childPt.containingTree = this.containingTree;

            childPt.SetupLine();

            childPt.curvatureControlPt = CurvatureControlPoint.AddPoint(this, childPt);

            return childPt;
        }

        public void RebuildFromSerialized(NodeSerialized root)
        {
            node.position = new Vector3(root.positionX, root.positionY, root.positionZ);
            node.radius = root.radius;

            children = new List<TreeControlPoint>();

            if (curvatureControlPt)
            {
                curvatureControlPt.transform.position = new Vector3(root.curvaturePointX,
                    root.curvaturePointY, root.curvaturePointZ);
            }

            foreach (NodeSerialized child in root.children)
            {
                TreeControlPoint childPt = AddNewChild();
                childPt.RebuildFromSerialized(child);
            }
        }

        public void SetMaterial(Material m)
        {
            meshRenderer.material = m;
        }

        public void SetCurvatureCenter(Vector3 center)
        {
            curvatureCenter = center;
        }

        /// <summary>
        /// Create telescoping segments from this control point to all child control points.
        /// 
        /// </summary>
        /// <returns></returns>
        public List<TelescopeElement> MakeTelescopesToChildren(List<TelescopeElement> allSegments, TelescopeElement parent)
        {
            TelescopeBulb rootBulb = TelescopeUtils.bulbOfRadius(node.position, node.radius);

            if (parent)
            {
                parent.ExtendImmediate(1);
                int parentNum = parent.numChildElements() - 1;
                rootBulb.SetParent(parent, parentNum, 1);
            }
            
            allSegments.Add(rootBulb);

            foreach (TreeControlPoint tcp in children)
            {
                // If the child is smaller, we make a forward-extending telescope.
                if (tcp.node.radius <= node.radius)
                {
                    TelescopeSegment childSegment = TelescopeUtils.telescopeOfCone(node.position, node.radius,
                        tcp.node.position, tcp.node.radius, tcp.curvatureCenter, useCurvature: tcp.hasCurvature);
                    childSegment.parent = rootBulb;
                    childSegment.parentElementNumber = 0;
                    allSegments.Add(childSegment);
                    tcp.MakeTelescopesToChildren(allSegments, childSegment);
                }
                // Otherwise we make a backward-extending telescope, and reverse it.
                else {
                    TelescopeSegment childSegment = TelescopeUtils.telescopeOfCone(tcp.node.position, tcp.node.radius,
                        node.position, node.radius, tcp.curvatureCenter, useCurvature: tcp.hasCurvature);
                    childSegment.ReverseTelescope();
                    childSegment.keepLocalPositionOnStart = true;
                    childSegment.parent = rootBulb;
                    childSegment.parentElementNumber = 0;
                    allSegments.Add(childSegment);
                    tcp.MakeTelescopesToChildren(allSegments, childSegment);
                }
            }
            return allSegments;
        }

        Vector3[] MakeArcPoints(int numPoints)
        {
            if (hasCurvature)
            {
                Vector3[] points = new Vector3[numPoints];

                float step = 1f / (numPoints - 1);

                Vector3 fromDir = parent.node.position - curvatureCenter;
                Vector3 toDir = node.position - curvatureCenter;

                float currentT = 0;

                for (int i = 0; i < numPoints; i++)
                {
                    Vector3 interpDiff = Vector3.Slerp(fromDir, toDir, currentT);
                    points[i] = curvatureCenter + interpDiff;
                    currentT += step;
                }

                return points;
            }
            else
            {
                Vector3[] points = { parent.node.position, node.position };
                return points;
            }
        }

        void Update()
        {
            if (lineRenderer && parent)
            {
                Vector3[] positions = MakeArcPoints(Constants.ARC_SAMPLES);
                lineRenderer.SetVertexCount(positions.Length);
                lineRenderer.SetPositions(positions);
            }
        }
    }

    public struct SphereNode
    {
        public float radius;
        public Vector3 position;

        public SphereNode(float r, Vector3 p)
        {
            radius = r;
            position = p;
        }

        public static Vector3 GetPosition(SphereNode n)
        {
            return n.position;
        }

        public static float GetRadius(SphereNode n)
        {
            return n.radius;
        }
    }
}