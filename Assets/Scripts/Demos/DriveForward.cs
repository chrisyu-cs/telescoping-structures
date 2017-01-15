using UnityEngine;
using System.Collections.Generic;

namespace Telescopes.Playground
{
    public class DriveForward : MonoBehaviour
    {
        public List<WheelScript> wheels;
        bool going = false;

        public float speed;

        float interp;

        // Update is called once per frame
        void Update()
        {
            if (Input.GetKeyDown("2"))
            {
                going = !going;

                if (going)
                {
                    foreach (WheelScript w in wheels) w.StartSpinning();
                }
                else
                {
                    foreach (WheelScript w in wheels) w.StopSpinning();
                }
            }

            transform.position += Vector3.back * Time.deltaTime * speed * interp;

            if (going)
            {
                interp = Mathf.MoveTowards(interp, 1, Time.deltaTime * 5f);
            }
            else
            {
                interp = Mathf.MoveTowards(interp, 0, Time.deltaTime * 5f);
            }
        }
    } 
}
