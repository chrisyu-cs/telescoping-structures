using UnityEngine;
using System.Collections;

namespace Telescopes.Playground
{
    public class CircumcenterTracker : MonoBehaviour
    {
        public Transform object1;
        public Transform object2;
        public Transform object3;
        
        // Update is called once per frame
        void Update()
        {
            Vector3 center = TelescopeUtils.Circumcenter(object1.position, object2.position, object3.position);
            transform.position = center;
        }
    }


}