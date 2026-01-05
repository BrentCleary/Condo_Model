using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Index : MonoBehaviour
{
    
    
    // Rooms
    List<Room> rooms = new List<Room>();

    // Add rooms
    rooms.Add(new Room("Living Room", 20, 15));
    rooms.Add(new Room("Bedroom", 15, 12));
    rooms.Add(new Room("Kitchen", 12, 10));
    rooms.Add(new Room("Bathroom", 8, 6));

    
    
    
    
    
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
