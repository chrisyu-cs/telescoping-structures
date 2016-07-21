using UnityEngine;
using System.Collections;

public class FrenetTracer : MonoBehaviour {

    public DiscreteCurve dCurve;
    public float arcLength;

    public bool bishop;

	// Use this for initialization
	void Start () {
        SetPositionOnCurve();
        bishop = false;
    }

    void SetPositionOnCurve()
    {
        if (!dCurve) return;
        transform.position = dCurve.PositionAtPoint(arcLength);
        OrthonormalFrame frame = bishop ? dCurve.BishopAtPoint(arcLength) : dCurve.FrenetAtPoint(arcLength);

        Quaternion r = Quaternion.LookRotation(frame.T, frame.B);
        transform.rotation = r;
    }
	
	// Update is called once per frame
	void Update () {
        if (dCurve)
        {
            arcLength = Mathf.Repeat(arcLength + Time.deltaTime, dCurve.ArcLength);
            SetPositionOnCurve();
        }
    }
}
