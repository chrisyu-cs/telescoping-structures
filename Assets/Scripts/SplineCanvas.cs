using UnityEngine;
using System.Collections.Generic;

using System;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Formatters.Binary;
using System.IO;

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
            if (bulbs == null) bulbs = new List<DraggablePoint>();
            if (splines == null) splines = new List<CatmullRomSpline>();

            dBulbs = new List<DCurveBulb>();
            dCurves = new List<DiscreteCurve>();

            tiBulbs = new List<DCurveBulb>();
            tiCurves = new List<TorsionImpulseCurve>();

            stage = CanvasStage.Spline;

            /*
            CatmullRomSpline crs = AddSpline(new Vector3(-2, 1, 0));
            crs.points.Add(new Vector3(-1, 1, 0));
            crs.points.Add(new Vector3(1, 1, 0));
            crs.points.Add(new Vector3(2, 1, 0));*/
        }

        public void Reset()
        {
            foreach (CatmullRomSpline crs in splines)
            {
                Destroy(crs.gameObject);
            }
            splines.Clear();

            foreach (DraggablePoint bulb in bulbs)
            {
                Destroy(bulb.gameObject);
            }
            bulbs.Clear();

            foreach (DiscreteCurve dc in dCurves)
            {
                Destroy(dc.gameObject);
            }
            dCurves.Clear();

            foreach (DCurveBulb bulb in dBulbs)
            {
                Destroy(bulb.gameObject);
            }
            dBulbs.Clear();

            foreach (TorsionImpulseCurve tic in tiCurves)
            {
                Destroy(tic.gameObject);
            }
            tiCurves.Clear();

            foreach (DCurveBulb bulb in tiBulbs)
            {
                Destroy(bulb.gameObject);
            }
            tiBulbs.Clear();
        }

        public void AddExistingSpline(CatmullRomSpline spline)
        {
            if (splines == null) splines = new List<CatmullRomSpline>();

            spline.transform.parent = transform;
            spline.containingCanvas = this;
            splines.Add(spline);
        }

        CatmullRomSpline AddSpline(Vector3 firstPos)
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
            return spline;
        }

        public DraggablePoint AddBulb(Vector3 pos)
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

            return point;
        }

        public void DeleteBulb(DraggablePoint bulb)
        {
            bulbs.Remove(bulb);
        }

        float RecentDepth()
        {
            return Camera.main.WorldToViewportPoint(mostRecentPoint).z;
        }


        public void DeleteSpline(CatmullRomSpline crs)
        {
            splines.Remove(crs);
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
            if (Input.GetKeyDown("page up"))
            {
                Debug.Log("up");
                SaveData("canvasTest.bin");
            }
            if (Input.GetKeyDown("page down"))
            {
                Debug.Log("down");
                ReloadFromFile("canvasTest.bin");
            }

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
                        impulseCurve.transform.parent = transform;

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

                    bool made = false;

                    foreach (DCurveBulb dcb in tiBulbs)
                    {
                        TelescopeBulb bulb = TelescopeUtils.bulbOfRadius(dcb.transform.position, dcb.radius);

                        if (!made)
                        {
                            made = true;
                            bulb.doOptimize = true;
                        }
                        bulbDict.Add(dcb, bulb);

                        float scaleAmount = Mathf.Sqrt(2);
                        bulb.Radius = bulb.Radius * scaleAmount;
                    }

                    foreach (TorsionImpulseCurve tic in tiCurves)
                    {
                        float startRadius = tic.NumSegments * Constants.DEFAULT_WALL_THICKNESS + 0.2f
                            + (tic.ArcLength * Constants.TAPER_SLOPE);

                        TelescopingSegment seg;

                        if (tic.StartBulb && tic.EndBulb)
                        {
                            if (tic.EndBulb.radius > tic.StartBulb.radius)
                            {
                                startRadius = tic.EndBulb.radius;
                                seg = tic.MakeTelescope(startRadius, reverse: true);
                            }
                            else
                            {
                                startRadius = tic.StartBulb.radius;
                                seg = tic.MakeTelescope(startRadius);
                            }
                        }
                        else if (tic.StartBulb && !tic.EndBulb)
                        {
                            startRadius = tic.StartBulb.radius;
                            seg = tic.MakeTelescope(startRadius);
                        }
                        else if (!tic.StartBulb && tic.EndBulb)
                        {
                            startRadius = tic.EndBulb.radius;
                            seg = tic.MakeTelescope(startRadius, reverse: true);
                        }
                        else
                        {
                            seg = tic.MakeTelescope(startRadius);
                        }

                        seg.ExtendImmediate(1);

                        if (tic.StartBulb)
                        {
                            TelescopeBulb bulb = bulbDict[tic.StartBulb];
                            seg.keepLocalPositionOnStart = true;
                            seg.SetParent(bulb);

                            bulb.childSegments.Add(seg);
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

                    foreach (TelescopeBulb bulb in bulbDict.Values)
                    {
                        Debug.Log("Bulb " + bulb.name + " has parent " + bulb.parentSegment
                            + " and " + bulb.childSegments.Count + " children");
                    }
                }
            }
        }

        public void SaveData(string filename)
        {
            SplineCanvasData scd = new SplineCanvasData();

            List<BulbData> bulbList = new List<BulbData>();
            for (int i = 0; i < bulbs.Count; i++)
            {
                bulbs[i].index = i;
                bulbList.Add(new BulbData(bulbs[i]));
            }

            List<SplineData> splineList = new List<SplineData>();
            foreach (CatmullRomSpline crs in splines)
            {
                splineList.Add(new SplineData(crs));
            }

            scd.bulbs = bulbList;
            scd.splines = splineList;

            IFormatter formatter = new BinaryFormatter();
            Stream stream = new FileStream(filename, FileMode.Create, FileAccess.Write, FileShare.None);
            formatter.Serialize(stream, scd);
            stream.Close();
        }

        public void SaveFromUI()
        {
            string name = DesignerController.instance.canvasFilenameField.text;
            SaveData(name);
        }

        public void ReloadFromUI()
        {
            string name = DesignerController.instance.canvasFilenameField.text;
            ReloadFromFile(name);
        }

        public void ReloadFromFile(string filename)
        {
            IFormatter formatter = new BinaryFormatter();
            Stream stream = new FileStream(filename, FileMode.Open, FileAccess.Read, FileShare.Read);
            SplineCanvasData data = (SplineCanvasData)formatter.Deserialize(stream);
            stream.Close();

            ReloadFromData(data);
        }

        public void ReloadFromData(SplineCanvasData data)
        {
            // Clear everything
            Reset();
            bulbs.Clear();

            foreach (BulbData bData in data.bulbs)
            {
                // Make bulbs at all the saved positions with the saved radii
                DraggablePoint pt = AddBulb(bData.position.GetVector());
                pt.SetSize(bData.radius);
                pt.containingCanvas = this;
            }

            foreach (SplineData sData in data.splines)
            {
                // Convert the serialized vectors to Unity vectors
                List<Vector3> posList = new List<Vector3>();
                foreach (V3Serialize v3s in sData.points)
                {
                    posList.Add(v3s.GetVector());
                }

                // Make a spline with the same control points
                CatmullRomSpline spline = CatmullRomSpline.SplineOfPoints(posList);
                AddExistingSpline(spline);

                // Set the begin/end bulbs
                if (sData.startBulb >= 0)
                {
                    spline.StartBulb = bulbs[sData.startBulb];
                }
                if (sData.endBulb >= 0)
                {
                    spline.EndBulb = bulbs[sData.endBulb];
                }
            }
        }
    }

    public enum CanvasStage
    {
        Spline, DCurve, ImpulseCurve, Telescope
    }
}