using UnityEngine;

using System;
using System.Collections.Generic;

namespace Telescopes
{

    [Serializable]
    public class SplineData
    {
        // The control points of this spline, stored in sequence.
        public List<V3Serialize> points;
        // The bulb attached to the start of this spline.
        public int startBulb;
        // The bulb attached to the end of this spline.
        public int endBulb;

        public SplineData(CatmullRomSpline spline)
        {
            points = new List<V3Serialize>();
            foreach (Vector3 v in spline.points)
            {
                points.Add(new V3Serialize(v));
            }

            startBulb = (spline.StartBulb) ? spline.StartBulb.index : -1;
            endBulb = (spline.EndBulb) ? spline.EndBulb.index : -1;
        } 
    }

    [Serializable]
    public class V3Serialize
    {
        public float x, y, z;

        public V3Serialize(Vector3 v)
        {
            x = v.x;
            y = v.y;
            z = v.z;
        }

        public Vector3 GetVector()
        {
            return new Vector3(x, y, z);
        }
    }

    [Serializable]
    public class BulbData
    {
        // The position of this bulb.
        public V3Serialize position;
        // The radius of this bulb.
        public float radius;

        public BulbData(DraggablePoint bulb)
        {
            position = new V3Serialize(bulb.transform.position);
            radius = bulb.radius;
        }
    }

    [Serializable]
    public class SplineCanvasData
    {
        public List<SplineData> splines;
        public List<BulbData> bulbs;
    }
}
