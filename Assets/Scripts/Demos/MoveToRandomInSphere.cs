using UnityEngine;
using System.Collections;

public class MoveToRandomInSphere : MonoBehaviour {

    public float minRadius = 4f;
    public float maxRadius = 10f;

    Vector3 velocity = Vector3.zero;
    Vector3 targetPos = Vector3.zero;
    bool moving = false;
	
	// Update is called once per frame
	void Update () {
	    if (Input.GetKeyDown("tab"))
        {
            Vector3 dir = Random.onUnitSphere;
            dir.y = Mathf.Abs(dir.y);
            float radius = Random.Range(minRadius, maxRadius);

            targetPos = dir * radius;
            moving = true;
        }

        if (moving)
        {
            transform.position = Vector3.SmoothDamp(transform.position, targetPos, ref velocity, 0.2f);
            if (Vector3.Distance(transform.position, targetPos) < 0.01f)
            {
                moving = false;
            }
        }
	}
}
