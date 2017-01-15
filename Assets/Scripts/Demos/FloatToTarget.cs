using UnityEngine;
using System.Collections;

public class FloatToTarget : MonoBehaviour {

    public Transform target;
    public bool activate = false;

    private Vector3 velocity;

	// Use this for initialization
	void Start () {
	
	}
	
	// Update is called once per frame
	void Update ()
    {
	    if (activate)
        {
            transform.position = Vector3.SmoothDamp(transform.position, target.position, ref velocity, 1f);
            transform.rotation = Quaternion.Slerp(transform.rotation, target.rotation, 0.1f);
        }
	}
}
