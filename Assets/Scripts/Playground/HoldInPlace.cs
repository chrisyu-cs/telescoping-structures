using UnityEngine;
using System.Collections;

public class HoldInPlace : MonoBehaviour {

    public Transform target;

    Vector3 targetPos;
    bool started = false;
	
	// Update is called once per frame
	void Update () {

        if (Input.GetKeyDown("1") && target)
        {
            targetPos = target.position;
            started = true;
            Debug.Log("Started pinning");
        }

        if (started)
        {
            Vector3 displacement = targetPos - target.position;
            transform.position = 0.9f * displacement;
        }
    }
}
