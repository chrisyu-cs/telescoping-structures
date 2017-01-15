using UnityEngine;
using System.Collections;

public class PushAndPull : MonoBehaviour {

    public Transform obj1;
    Vector3 initDisp;
    float posParameter;
    float targetParameter;

    public float lowerBound = 0f;

	// Use this for initialization
	void Start ()
    {
        initDisp = obj1.position - transform.position;
        posParameter = 1;
        targetParameter = 1;
	}

    void SetDisplacement(float t)
    {
        Vector3 v = obj1.transform.position;
        v.z = transform.position.z + t * initDisp.z;
        obj1.transform.position = v;
    }
	
	// Update is called once per frame
	void Update ()
    {
	    if (Input.GetKeyDown("0"))
        {
            targetParameter = lowerBound;
        }
        if (Input.GetKeyDown("1"))
        {
            targetParameter = 1;
        }

        posParameter = Mathf.MoveTowards(posParameter, targetParameter, Time.deltaTime);

        SetDisplacement(posParameter);
	}
}
