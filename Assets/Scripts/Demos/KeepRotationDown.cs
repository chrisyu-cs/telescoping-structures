using UnityEngine;
using System.Collections;

namespace Telescopes.Playground
{
    public class KeepRotationDown : MonoBehaviour
    {

        Quaternion initRotation;

        void Start()
        {
            initRotation = transform.rotation;
        }
        // Update is called once per frame
        void Update()
        {
            transform.rotation = initRotation;
        }
    }

}