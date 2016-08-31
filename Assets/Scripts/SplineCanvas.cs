using UnityEngine;
using System.Collections.Generic;

namespace Telescopes
{
    public class SplineCanvas : MonoBehaviour
    {
        List<CatmullRomSpline> splines;
        List<DraggablePoint> bulbs;

        List<DCurveBulb> dBulbs;
        List<DiscreteCurve> dCurves;

        List<DCurveBulb> tiBulbs;
        List<TorsionImpulseCurve> tiCurves;

        bool tiCreated = false;

        public Vector3 mostRecentPoint;

        CanvasStage stage;

        // Use this for initialization
        void Start()
        {
            bulbs = new List<DraggablePoint>();
            
            splines = new List<CatmullRomSpline>();

            dBulbs = new List<DCurveBulb>();
            dCurves = new List<DiscreteCurve>();

            tiBulbs = new List<DCurveBulb>();
            tiCurves = new List<TorsionImpulseCurve>();

            stage = CanvasStage.Spline;
        }

        public void AddExistingSpline(CatmullRomSpline spline)
        {
            if (splines == null) splines = new List<CatmullRomSpline>();

            spline.transform.parent = transform;
            spline.containingCanvas = this;
            splines.Add(spline);
        }

        void AddSpline(Vector3 firstPos)
        {
            GameObject splineObj = new GameObject();
            splineObj.transform.parent = transform;

            mostRecentPoint = firstPos;
            CatmullRomSpline spline = splineObj.AddComponent<CatmullRomSpline>();
            spline.points = new List<Vector3>();
            spline.points.Add(firstPos);
            spline.containingCanvas = this;
            splines.Add(spline);
            spline.SetMaterial(DesignerController.instance.defaultLineMaterial);

            splineObj.name = "spline" + splines.Count;
        }

        void AddBulb(Vector3 pos)
        {
            GameObject bulbObj = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            bulbObj.transform.position = pos;
            bulbObj.transform.parent = transform;

            DraggablePoint point = bulbObj.AddComponent<DraggablePoint>();
            point.Type = PointType.Bulb;
            point.containingCanvas = this;
            point.radius = 0.5f;

            bulbs.Add(point);
            bulbObj.name = "bulb" + bulbs.Count;
        }

        float RecentDepth()
        {
            return Camera.main.WorldToViewportPoint(mostRecentPoint).z;
        }

        public DraggablePoint IntersectedBulb(Vector3 point)
        {
            foreach (DraggablePoint bulb in bulbs)
            {
                Vector3 center = bulb.transform.position;
                float distance = Vector3.Distance(point, center);
                if (distance <= bulb.radius) return bulb;
            }
            return null;
        }

        void Update()
        {
            if (stage == CanvasStage.Spline)
            {
                if (Input.GetKey("left shift") && Input.GetKeyDown("b"))
                {
                    Vector3 mouse = Input.mousePosition;
                    mouse.z = RecentDepth();
                    Vector3 worldPos = Camera.main.ScreenToWorldPoint(mouse);
                    AddBulb(worldPos);
                }

                if (Input.GetKey("left shift") && Input.GetKeyDown("n"))
                {
                    Vector3 mouse = Input.mousePosition;
                    mouse.z = RecentDepth();
                    Vector3 worldPos = Camera.main.ScreenToWorldPoint(mouse);
                    AddSpline(worldPos);
                }

                if (Input.GetKey("left shift") && Input.GetKeyDown("p"))
                {
                    Dictionary<DraggablePoint, DCurveBulb> bulbDict = new Dictionary<DraggablePoint, DCurveBulb>();

                    foreach (DraggablePoint bulb in bulbs)
                    {
                        DCurveBulb dcBulb = bulb.ConvertToBulb();
                        dBulbs.Add(dcBulb);
                        bulbDict.Add(bulb, dcBulb);
                    }

                    foreach (CatmullRomSpline spline in splines)
                    {
                        DiscreteCurve dc = spline.ConvertToDCurve();

                        if (spline.StartBulb)
                        {
                            DCurveBulb parentOfSpline = bulbDict[spline.StartBulb];
                            dc.parentBulb = parentOfSpline;
                            parentOfSpline.childCurves.Add(dc);
                        }
                        if (spline.EndBulb)
                        {
                            DCurveBulb endOfSpline = bulbDict[spline.EndBulb];
                            dc.childBulb = endOfSpline;
                            endOfSpline.parentCurve = dc;
                        }

                        dCurves.Add(dc);
                    }

                    stage = CanvasStage.DCurve;
                }
            }

            else if (stage == CanvasStage.DCurve)
            {
                if (Input.GetKey("left shift") && Input.GetKeyDown("o"))
                {
                    // Clear away the old stuff
                    foreach (TorsionImpulseCurve tic in tiCurves)
                    {
                        Destroy(tic.gameObject);
                    }
                    tiCurves.Clear();

                    foreach (DCurveBulb dcb in tiBulbs)
                    {
                        Destroy(dcb.gameObject);
                    }
                    tiBulbs.Clear();

                    Dictionary<DCurveBulb, DCurveBulb> bulbDict = new Dictionary<DCurveBulb, DCurveBulb>();

                    foreach (DCurveBulb origBulb in dBulbs)
                    {
                        DCurveBulb newBulb = origBulb.Duplicate();
                        tiBulbs.Add(newBulb);
                        bulbDict.Add(origBulb, newBulb);
                    }

                    foreach (DiscreteCurve dc in dCurves)
                    {
                        TorsionImpulseCurve impulseCurve = dc.MakeCurve();
                        tiCurves.Add(impulseCurve);

                        if (dc.parentBulb)
                        {
                            DCurveBulb newParent = bulbDict[dc.parentBulb];
                            newParent.childCurves.Add(impulseCurve);
                            impulseCurve.StartBulb = newParent;
                        }

                        if (dc.childBulb)
                        {
                            DCurveBulb newChild = bulbDict[dc.childBulb];
                            newChild.parentCurve = impulseCurve;
                            impulseCurve.EndBulb = newChild;
                        }
                    }

                    tiCreated = true;
                }

                else if (Input.GetKey("left shift") && Input.GetKeyDown("return") && tiCreated)
                {
                    stage = CanvasStage.ImpulseCurve;

                    foreach (DCurveBulb db in dBulbs)
                    {
                        db.gameObject.SetActive(false);
                    }
                    foreach (DiscreteCurve dc in dCurves)
                    {
                        dc.gameObject.SetActive(false);
                    }
                }
            }

            else if (stage == CanvasStage.ImpulseCurve)
            {
                if (Input.GetKey("left shift") && Input.GetKeyDown("i"))
                {
                    Dictionary<DCurveBulb, TelescopeBulb> bulbDict = new Dictionary<DCurveBulb, TelescopeBulb>();

                    foreach (DCurveBulb dcb in tiBulbs)
                    {
                        TelescopeBulb bulb = TelescopeUtils.bulbOfRadius(dcb.transform.position, dcb.radius);
                        bulbDict.Add(dcb, bulb);
                    }

                    foreach (TorsionImpulseCurve tic in tiCurves)
                    {
                        TelescopingSegment seg = tic.MakeTelescope(tic.NumSegments * Constants.DEFAULT_WALL_THICKNESS + 0.1f);
                        seg.ExtendImmediate(1);

                        if (tic.StartBulb)
                        {
                            TelescopeBulb bulb = bulbDict[tic.StartBulb];
                            seg.keepLocalPositionOnStart = true;
                            seg.SetParent(bulb);
                        }
                        if (tic.EndBulb)
                        {
                            TelescopeBulb bulb = bulbDict[tic.EndBulb];
                            bulb.keepLocalPositionOnStart = true;
                            bulb.SetParentToSegmentEnd(seg);
                        }

                        foreach (DCurveBulb oldBulb in tiBulbs)
                        {
                            oldBulb.gameObject.SetActive(false);
                        }

                        foreach (TorsionImpulseCurve oldTc in tiCurves)
                        {
                            oldTc.gameObject.SetActive(false);
                        }
                    }

                    stage = CanvasStage.Telescope;
                }
            }
        }
    }

    public enum CanvasStage
    {
        Spline, DCurve, ImpulseCurve, Telescope
    }
}