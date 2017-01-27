using UnityEngine;
using System.Collections.Generic;
using Telescopes;

namespace Telescopes.Playground
{
    public class OctoMaker : MonoBehaviour
    {
        public Mesh octoTop;
        public Material octoMaterial;

        public SplineCanvas canvas;
        int counter = 0;
        int frame = 0;

        // Use this for initialization
        void Start()
        {

        }

        // Update is called once per frame
        void Update()
        {
            frame++;
            if (frame > 0 && counter == 0)
            {
                canvas.ReloadFromFile("octo2.canvas");
                counter++;
            }
            if (frame > 10 && counter == 1)
            {
                canvas.SplineToDCurve();
                counter++;
            }
            if (frame > 20 && counter == 2)
            {
                canvas.DCurveToTIC();
                counter++;
            }
            if (frame > 30 && counter == 3)
            {
                canvas.ConfirmTIC();
                canvas.TICToTelescope();
                counter++;
            }

            if (frame > 40 && counter == 4)
            {
                canvas.ActiveStructure.transform.rotation *= Quaternion.AngleAxis(180, Vector3.up);

                foreach (TelescopeJuncture j in canvas.ActiveStructure.junctures)
                {
                    j.customMesh = octoTop;
                    j.junctureType = JunctureType.CustomMesh;
                    j.SetMaterial(octoMaterial);
                }
                
                foreach (TelescopeSegment seg in canvas.ActiveStructure.segments)
                {
                    foreach (TelescopeShell shell in seg.shells)
                    {
                        shell.setMaterial(octoMaterial);
                    }
                }

                KeepOnGround keep = canvas.ActiveStructure.gameObject.AddComponent<KeepOnGround>();
                keep.groundLevel = 1f;
                keep.targets = new List<Transform>();

                foreach (TelescopeSegment seg in canvas.ActiveStructure.segments)
                {
                    keep.targets.Add(seg.LastShell.transform);
                }

                counter++;
            }

            if (frame > 50 && counter == 5)
            {
                int[] indices = { 3, 4, 1, 0, 5, 2 };

                Vector3 initialPoint = 4.5f * Vector3.right + 1f * Vector3.up;

                foreach (int i in indices)
                {
                    GameObject obj = new GameObject();
                    obj.transform.position = initialPoint;

                    JitterPosition j = obj.AddComponent<JitterPosition>();
                    j.changeInterval = 1f;
                    j.radius = 4f;
                    j.startOffset = Random.Range(0, j.changeInterval);

                    initialPoint = Quaternion.AngleAxis(-60f, Vector3.up) * initialPoint;

                    TelescopeSegment seg = canvas.ActiveStructure.segments[i];
                    TelescopeIK ik = seg.gameObject.AddComponent<TelescopeIK>();

                    ik.target = obj.transform;
                }

                counter++;
            }
        }
    }

}