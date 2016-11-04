using UnityEngine;
using System.Collections.Generic;

using UnityDDG;

namespace Telescopes.Playground
{
    public class ConvHullTester : MonoBehaviour
    {
        public int numPoints = 500;
        List<GameObject> spheres;

        void MakeSphere(Vector3 pos)
        {
            GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.transform.localScale = new Vector3(0.1f, 0.1f, 0.1f);
            sphere.transform.position = pos;

            spheres.Add(sphere);
        }

        // Use this for initialization
        void Start()
        {
            spheres = new List<GameObject>();

            for (int i = 0; i < numPoints; i++)
            {
                Vector3 pos = Random.insideUnitSphere + Vector3.up;
                pos *= 2;

                MakeSphere(pos);
            }
        }

        static Vector3 PosOfObj(GameObject g)
        {
            return g.transform.position;
        }

        // Update is called once per frame
        void Update()
        {
            
            if (Input.GetKeyDown("]"))
            {
                List<Vector3> positions = spheres.ConvertAll<Vector3>(new System.Converter<GameObject, Vector3>(PosOfObj));
                Mesh m = GeometryUtils.ConvexHull3D(positions);

                foreach (GameObject g in spheres)
                {
                    Destroy(g);
                }
                spheres.Clear();

                GameObject meshObj = new GameObject();
                meshObj.transform.parent = transform;

                MeshFilter mf = meshObj.AddComponent<MeshFilter>();
                MeshRenderer mr = meshObj.AddComponent<MeshRenderer>();
                mr.material = DesignerController.instance.defaultTelescopeMaterial;

                mf.mesh = m;
            }
        }
    }

}