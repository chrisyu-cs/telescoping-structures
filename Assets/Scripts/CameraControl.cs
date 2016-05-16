using UnityEngine;
using System.Collections;

namespace Telescopes
{
    public class CameraControl : MonoBehaviour
    {

        bool locked;
        private float theta, phi;
        public float translateSpeed = 2f;

        // Use this for initialization
        void Start()
        {
            locked = false;
            theta = 0;
            phi = 0;
        }

        // Update is called once per frame
        void Update()
        {
            if (Input.GetKeyDown("f"))
            {
                locked = !locked;
            }

            float up = Input.GetAxis("Vertical");
            float right = Input.GetAxis("Horizontal");
            float lmb = Input.GetAxis("Fire1");
            float rmb = Input.GetAxis("Fire3");

            transform.position += up * transform.forward * Time.deltaTime * translateSpeed;
            transform.position += right * transform.right * Time.deltaTime * translateSpeed;
            transform.position += rmb * transform.up * Time.deltaTime * translateSpeed;
            transform.position += lmb * -transform.up * Time.deltaTime * translateSpeed;
            
            if (!locked)
            {
                float mouseX = Input.GetAxis("Mouse X");
                float mouseY = -Input.GetAxis("Mouse Y");

                theta = Mathf.Repeat(theta + mouseX, 360);
                phi = Mathf.Clamp(phi + mouseY, -90, 90);
            }

            transform.rotation = Quaternion.Euler(phi, theta, 0);
        }
    }

}