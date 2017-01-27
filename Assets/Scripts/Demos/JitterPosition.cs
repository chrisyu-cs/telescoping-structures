using UnityEngine;
using System.Collections;

public class JitterPosition : MonoBehaviour {

    public float changeInterval = 1f;
    public float startOffset = 0f;

    float time;
    Vector3 target;
    Vector3 original;

    public float radius = 1f;

    Vector3 velocity;

	// Use this for initialization
	void Start ()
    {
        time = startOffset;
        original = transform.position;
        target = original;
    }
	
	// Update is called once per frame
	void Update ()
    {
        time += Time.deltaTime;
	    if (time > changeInterval)
        {
            time = 0;
            target = original + radius * Random.insideUnitSphere;
        }

        transform.position = Vector3.SmoothDamp(transform.position, target, ref velocity, changeInterval);
	}
}
