using UnityEngine;
using System.Collections;

public class ClawCloser : MonoBehaviour {
    
    private Animator animator;
    bool open = true;

    // Use this for initialization
    void Start()
    {
        animator = GetComponent<Animator>();
    }

    public bool ToggleClose()
    {
        open = !open;
        animator.SetTrigger("toggle-close");
        return open;
    }
}
