using UnityEngine;
using System.Collections.Generic;

public class KeepOnGround : MonoBehaviour {

    public float groundLevel = 0f;

    public List<Transform> targets;
	
	// Update is called once per frame
	void Update ()
    {
        float minY = 100;
	    foreach (Transform t in targets)
        {
            minY = Mathf.Min(t.position.y, minY);
        }

        float correction = groundLevel - minY;

        transform.position += correction * Vector3.up;
	}
}
