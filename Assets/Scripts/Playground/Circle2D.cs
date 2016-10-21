using UnityEngine;
using System.Collections.Generic;

using Telescopes;

namespace Telescopes.Playground
{
    [RequireComponent(typeof(LineRenderer))]
    public class Circle2D : MonoBehaviour
    {
        public float radius;
        LineRenderer lineRender;

        public Vector3 Center
        {
            get
            {
                return transform.position;
            }
        }

        public Vector3 Normal
        {
            get
            {
                return transform.up;
            }
        }

        // Use this for initialization
        void Start()
        {
            lineRender = GetComponent<LineRenderer>();
            lineRender.useWorldSpace = false;

            lineRender.SetWidth(0.1f, 0.1f);
            lineRender.material = DesignerController.instance.defaultLineMaterial;

            SetUpCircle();
        }

        void SetUpCircle()
        {
            int numPoints = 80;
            float angleStep = 2 * Mathf.PI / numPoints;

            List<Vector3> points = new List<Vector3>();

            for (int i = 0; i < numPoints + 1; i++)
            {
                Vector3 p = new Vector3(Mathf.Cos(i * angleStep), 0, Mathf.Sin(i * angleStep));
                points.Add(p * radius);
            }

            lineRender.SetVertexCount(points.Count);
            lineRender.SetPositions(points.ToArray());
        }

        // Update is called once per frame
        void Update()
        {

        }
    }

}