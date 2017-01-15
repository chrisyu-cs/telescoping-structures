using UnityEngine;
using System.Collections.Generic;

using Telescopes;

namespace Telescopes.Playground
{
    public class BracMaker : MonoBehaviour
    {
        public SplineCanvas canvas;
        int counter = 0;
        int frame = 0;

        public WheelScript wheelPrefab;

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
                canvas.ReloadFromFile("brac4.canvas");
                counter++;
            }
            if (frame > 10 && counter == 1)
            {
                canvas.SplineToDCurve();
                counter++;

                foreach (DiscreteCurve dc in FindObjectsOfType<DiscreteCurve>())
                {
                    if (dc.name == "DiscretizedCurve1" ||
                        dc.name == "DiscretizedCurve2" ||
                        dc.name == "DiscretizedCurve3" ||
                        dc.name == "DiscretizedCurve4")
                    {
                        dc.BulbShrinkRatio = 0.5f;
                    }

                    else if (dc.name == "DiscretizedCurve5")
                    {
                        dc.BulbShrinkRatio = 0.8f;
                    }
                }
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
                foreach (TelescopeJuncture j in canvas.ActiveStructure.junctures)
                {
                    j.CollisionIteration(0.5f);
                    j.MakeMesh();
                }
                foreach (TelescopeJuncture j in canvas.ActiveStructure.junctures)
                {
                    j.CollisionIteration(0.5f);
                    j.MakeMesh();

                    if (j.name == "bulb2" || j.name == "bulb3" || j.name == "bulb5" || j.name == "bulb6")
                    {
                        j.junctureType = JunctureType.None;
                        j.transform.FindChild("Sphere").gameObject.SetActive(false);
                        j.GetComponent<LineRenderer>().enabled = false;
                    }
                }
                counter++;
            }

            if (frame > 50 && counter == 5)
            {

                DriveOnGround d = canvas.ActiveStructure.gameObject.AddComponent<DriveOnGround>();

                foreach (TelescopeJuncture j in canvas.ActiveStructure.junctures)
                {
                    if (j.name == "bulb2" || j.name == "bulb3" || j.name == "bulb5" || j.name == "bulb6")
                    {
                        WheelScript wheelObj = Instantiate<WheelScript>(wheelPrefab);
                        Vector3 pos = j.transform.position;
                        pos.y = 0;
                        wheelObj.transform.position = pos;
                        wheelObj.gameObject.AddComponent<KeepRotationDown>();
                        wheelObj.transform.parent = j.transform;
                        wheelObj.reverse = false;

                        Destroy(wheelObj.GetComponent<ParentToLastShell>());

                        switch (j.name)
                        {
                            case "bulb2": d.frontRight = wheelObj; break;
                            case "bulb3": d.frontLeft = wheelObj; break;
                            case "bulb6": d.backLeft = wheelObj; break;
                            case "bulb5": d.backRight = wheelObj; break;
                            default: break;
                        }
                    }
                }

                d.levelY = 0.23f;

                counter++;
            }
        }
    }

}