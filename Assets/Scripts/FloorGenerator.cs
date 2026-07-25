using System.Collections.Generic;
using UnityEngine;

public class FloorGenerator : MonoBehaviour
{

	// Start is called once before the first execution of Update after the MonoBehaviour is created

	void Start()
	{
		// Make sure the list has been populated (call this once)
		// You can move the creation code into a public static method on RoomIndex if you prefer
		if (RoomIndex.FloorSectionList.Count == 0)
		{
			// Temporary: force the initialization that used to be in RoomIndex.Start()
			// Better long-term: extract that code into a public static Initialize() method
			new RoomIndex().Start();   // works only because Start is public
		}

		// Example: generate the Kitchen by looking it up in the list
		RoomIndex.FloorSection livingRoom = RoomIndex.FloorSectionList
				.Find(s => s.Name == "LivingRoom");

		if (livingRoom != null)
			GenerateFloor(livingRoom);
	}




  public void GenerateFloor(RoomIndex.FloorSection floor)
	{
		float width  = floor.Width;
		float length = floor.Length;

		GameObject floorTile = GameObject.CreatePrimitive(PrimitiveType.Cube);
		
		// Move the center so the lowest-left corner lands on SpawnPos
		// (X = width, Z = length, Y = thickness)
		floorTile.transform.localPosition = floor.SpawnPos
																			+ new Vector3(width * 0.5f, 0.05f, length * 0.5f);
		floorTile.transform.localScale = new Vector3(width, 0.1f, length);
		floorTile.GetComponent<Renderer>().material.color = Color.blueViolet;

	}



}
