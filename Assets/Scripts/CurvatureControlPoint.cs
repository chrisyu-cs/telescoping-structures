using UnityEngine;
using System.Collections;

namespace Telescopes
{
    public class CurvatureControlPoint : MonoBehaviour
    {
        public TreeControlPoint parentControlPoint;
        public TreeControlPoint childControlPoint;

        private GameObject sphere;

        public static CurvatureControlPoint AddPoint(TreeControlPoint parent, TreeControlPoint child)
        {
            GameObject pointObj = new GameObject();
            pointObj.transform.parent = parent.containingTree.transform;

            Vector3 midpoint = (parent.transform.position + child.transform.position) / 2;
            pointObj.transform.position = midpoint;

            CurvatureControlPoint ccp = pointObj.AddComponent<CurvatureControlPoint>();
            ccp.parentControlPoint = parent;
            ccp.childControlPoint = child;

            return ccp;
        }

        void Start()
        {
            sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.transform.parent = transform;
            sphere.transform.localPosition = Vector3.zero;

            sphere.transform.localScale = new Vector3(0.3f, 0.3f, 0.3f);

            SphereCollider c = sphere.GetComponent<SphereCollider>();
            Destroy(c);
            SphereCollider sphereCollider = gameObject.AddComponent<SphereCollider>();
            sphereCollider.center = Vector3.zero;
            sphereCollider.radius = 0.15f;

            gameObject.layer = 10;
        }

        // Update is called once per frame
        void Update()
        {
            // Restrict this point to the smallest sphere containing both endpoints.
            Vector3 boundingCenter = (parentControlPoint.node.position + childControlPoint.node.position) / 2;
            float boundingRadius = Vector3.Distance(parentControlPoint.node.position, childControlPoint.node.position) / 2;

            Vector3 diffFromCenter = transform.position - boundingCenter;
            float distance = diffFromCenter.magnitude;
            if (distance > boundingRadius)
            {
                diffFromCenter.Normalize();
                transform.position = boundingCenter + 0.99f * boundingRadius * diffFromCenter;
            }

            if (TelescopeUtils.IsColinear(parentControlPoint.node.position,
                childControlPoint.node.position, transform.position))
            {
                childControlPoint.hasCurvature = false;
            }
            else
            {
                childControlPoint.hasCurvature = true;
                // Set the curvature of the arc from child to parent.
                Vector3 circumcenter = TelescopeUtils.Circumcenter(parentControlPoint.node.position,
                    childControlPoint.node.position, transform.position);
                
                childControlPoint.SetCurvatureCenter(circumcenter);
            }
        }
    }

}