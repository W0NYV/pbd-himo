
using UdonSharp;
using UnityEngine;
using VRC.SDKBase;
using VRC.Udon;

public class Cube : UdonSharpBehaviour {
    
    public float speed;

    void Start() {
        
    }

    void Update() {

        this.gameObject.transform.Rotate(this.gameObject.transform.up * speed * Time.deltaTime);
        
    }

}
