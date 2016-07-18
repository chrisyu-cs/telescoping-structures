using UnityEngine;
using System.Collections;

public class LimitedLifetime : MonoBehaviour {

    public float lifetime;
    private float age;

	// Use this for initialization
	void Start () {
        age = 0;
	}
	
	// Update is called once per frame
	void Update () {
        age += Time.deltaTime;
        if (age > lifetime)
        {
            Destroy(gameObject);
        }
	}
}
