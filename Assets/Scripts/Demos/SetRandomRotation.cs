using UnityEngine;
using System.Collections;

public class SetRandomRotation : MonoBehaviour {

	// Use this for initialization
	void Start ()
    {
        Quaternion q = Random.rotationUniform;
        transform.rotation = q;

        float scaleFactor = Random.Range(2, 4);
        transform.localScale = new Vector3(scaleFactor, scaleFactor, scaleFactor);
	}
}
