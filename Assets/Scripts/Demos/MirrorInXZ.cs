using UnityEngine;
using System.Collections;

public class MirrorInXZ : MonoBehaviour {

    public float level;
    public Transform target;
    
	// Update is called once per frame
	void Update () {
        Vector3 p = target.position;
        p.y = level;
        transform.position = p;
	}
}
