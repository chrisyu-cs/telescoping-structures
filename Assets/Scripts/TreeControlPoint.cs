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

        private static int nodeNum = 0;
        private GameObject sphere;
        private SphereCollider sphereCollider;
        private LineRenderer lineRenderer;
        private MeshRenderer meshRenderer;

        void Start()
        {
            children = new List<TreeControlPoint>();
            node = new SphereNode(Constants.INITIAL_SPLINE_SIZE, transform.position);
            sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.transform.parent = transform;
            sphere.transform.position = node.position;

            meshRenderer = sphere.GetComponent<MeshRenderer>();

            SphereCollider c = sphere.GetComponent<SphereCollider>();
            Destroy(c);
            sphereCollider = gameObject.AddComponent<SphereCollider>();
            sphereCollider.center = Vector3.zero;

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

        public void AddNewChild()
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
        }

        public void SetMaterial(Material m)
        {
            meshRenderer.material = m;
        }

        /// <summary>
        /// Create telescoping segments from this control point to all child control points.
        /// This is done by first adding a bulb at this point's location with the given radius.
        /// 
        /// </summary>
        /// <returns></returns>
        public List<TelescopingSegment> MakeTelescopesToChildren()
        {
            return null;
        }

        void Update()
        {
            if (lineRenderer && parent)
            {
                Vector3[] positions = { parent.node.position, node.position };
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