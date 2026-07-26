using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;

public class RoomIndex
{
	// Walls - inches
	private static int _nextID = 1;

	public List<string> FloorType = new List<string>() { "LVP", "Carpet", "Tile"};
	public static List<FloorSection> FloorSectionList = new List<FloorSection>();

	// ----- ----- ----- ----- ----- ----- ----- FloorSection Class ----- ----- ----- ----- ----- ----- ----- 

	public class FloorSection{

		public	int			ID;
		public	string	Name; 
		public	int			Level;
		public  Vector3	SpawnPos;
		public	float		Width;  // North - South 
		public	float		Length; // East - West
		public	float		Sqft;
		private float		sqftConversion = 144f; // 12x12 inches
		public	string	FloorType;
		public  bool    IsActive;

		// -----  Constructor ----- 
		public FloorSection(string name, int level, Vector3 spawnPos, float width, float length, string floorType, bool isActive = true)
		{
			ID				= _nextID;
			_nextID		= _nextID + 1;

			Name			= name;
			Level			= level;
			SpawnPos	= spawnPos;
			Width			= width;
			Length		= length;
			Sqft			= length * width / sqftConversion;
			FloorType = floorType;
			IsActive  = isActive;

			FloorSectionList.Add(this);
		}
	}

	

	Vector3 EntryWay_Pos = new Vector3(); 
	Vector3 Hallway_Pos = new Vector3();
	Vector3 Bathroom_Pos = new Vector3();
	Vector3 Bedroom_Pos = new Vector3();
	Vector3 BedroomCloset_Pos = new Vector3();
	Vector3 StoreRoom_Pos = new Vector3();
	Vector3 Kitchen_Pos = new Vector3();
	Vector3 Laundry_Pos = new Vector3();
	Vector3 Loft_Pos = new Vector3();
	Vector3 SecondBedroom_Pos = new Vector3();
	Vector3 SecondBedroomCloset_Pos = new Vector3();

	// ----- ----- ----- ----- ----- ----- ----- Initializer ----- ----- ----- ----- ----- ----- -----

	public static FloorSection EntryWay							= new FloorSection("EntryWay",						1, Vector3.zero, 40.25f, 43f, "LVP");
	public static FloorSection EntryCloset					= new FloorSection("EntryCloset",					1, Vector3.zero, 35f, 57f, "LVP");
	public static FloorSection Hallway							= new FloorSection("Hallway",							1, Vector3.zero, 229f, 36f, "LVP");
	public static FloorSection Bathroom							= new FloorSection("Bathroom",						1, Vector3.zero, 63.75f, 71.25f, "LVP");
	public static FloorSection Bedroom							= new FloorSection("Bedroom",							1, Vector3.zero, 157f, 148f, "LVP");
	public static FloorSection BedroomCloset				= new FloorSection("BedroomCloset",				1, Vector3.zero, 78f, 71f, "LVP");
	public static FloorSection StoreRoom						= new FloorSection("Storeroom",						1, Vector3.zero, 126f, 200f, "Carpet");
	public static FloorSection Kitchen							= new FloorSection("Kitchen",							1, Vector3.zero, 114f, 90.5f, "LVP");
	public static FloorSection Laundry							= new FloorSection("Laundry",							1, Vector3.zero, 31.25f, 89.5f, "LVP");
	public static FloorSection LivingRoom						= new FloorSection("LivingRoom",					1, Vector3.zero, 151.75f, 152f, "LVP");
	public static FloorSection DiningRoom						= new FloorSection("DiningRoom",					1, Vector3.zero, 125.75f, 95.25f, "LVP");
	public static FloorSection Loft									= new FloorSection("Loft",								2, Vector3.zero, 183f, 137f, "Carpet");
	public static FloorSection SecondBedroom				= new FloorSection("SecondBedroom",				2, Vector3.zero, 130.5f, 165f, "Carpet");
	public static FloorSection SecondBedroomCloset	= new FloorSection("SecondBedroomCloset", 2, Vector3.zero, 76f, 23.75f, "Carpet");




	public void Start()
	{
		Calculate_Vertices();
	}




	public void Calculate_Vertices()
	{

		// LivingRoom_Vertices;
		Vector3 LivingRoom_SpawnPos = new Vector3(0, 1, 0); // Starting Reference (Furthest North-West Corner of the House)

		Vector3 LivingRoom_TopLeft			= new Vector3(LivingRoom_SpawnPos.x,										 LivingRoom_SpawnPos.y, LivingRoom_SpawnPos.z + LivingRoom.Length);
		Vector3 LivingRoom_TopRight			= new Vector3(LivingRoom_SpawnPos.x + LivingRoom.Width, LivingRoom_SpawnPos.y, LivingRoom_SpawnPos.z + LivingRoom.Length);
		Vector3 LivingRoom_BottomLeft		= new Vector3(LivingRoom_SpawnPos.x,										 LivingRoom_SpawnPos.y, LivingRoom_SpawnPos.z);
		Vector3 LivingRoom_BottomRight	= new Vector3(LivingRoom_SpawnPos.x + LivingRoom.Width, LivingRoom_SpawnPos.y, LivingRoom_SpawnPos.z);

		//DiningRoom_Vertices()
		Vector3 DiningRoom_SpawnPos = LivingRoom_TopLeft;

		Vector3 DiningRoom_TopLeft			= new Vector3(DiningRoom_SpawnPos.x,										 DiningRoom_SpawnPos.y, DiningRoom_SpawnPos.z + DiningRoom.Length);
		Vector3 DiningRoom_TopRight			= new Vector3(DiningRoom_SpawnPos.x + DiningRoom.Width, DiningRoom_SpawnPos.y, DiningRoom_SpawnPos.z + DiningRoom.Length);
		Vector3 DiningRoom_BottomLeft		= new Vector3(DiningRoom_SpawnPos.x,										 DiningRoom_SpawnPos.y, DiningRoom_SpawnPos.z);
		Vector3 DiningRoom_BottomRight	= new Vector3(DiningRoom_SpawnPos.x + DiningRoom.Width, DiningRoom_SpawnPos.y, DiningRoom_SpawnPos.z);

		//Bedroom_Vertices()
		Vector3 Bedroom_SpawnPos				= new Vector3(LivingRoom_BottomRight.x, 1, LivingRoom_BottomRight.z - Hallway.Length);

		Vector3 Bedroom_TopLeft					= new Vector3(Bedroom_SpawnPos.x,									 Bedroom_SpawnPos.y, Bedroom_SpawnPos.z + Bedroom.Length);
		Vector3 Bedroom_TopRight				= new Vector3(Bedroom_SpawnPos.x + Bedroom.Width, Bedroom_SpawnPos.y, Bedroom_SpawnPos.z + Bedroom.Length);
		Vector3 Bedroom_BottomLeft			= new Vector3(Bedroom_SpawnPos.x,									 Bedroom_SpawnPos.y, Bedroom_SpawnPos.z);
		Vector3 Bedroom_BottomRight			= new Vector3(Bedroom_SpawnPos.x + Bedroom.Width, Bedroom_SpawnPos.y, Bedroom_SpawnPos.z);

		//Hallway_Vertices()
		Vector3 Hallway_SpawnPos				= new Vector3(LivingRoom_TopRight.x, 1, LivingRoom_TopRight.z - Hallway.Length);

		Vector3 Hallway_TopLeft					= new Vector3(Hallway_SpawnPos.x,									Hallway_SpawnPos.y, Hallway_SpawnPos.z + Hallway.Length);
		Vector3 Hallway_TopRight				= new Vector3(Hallway_SpawnPos.x + Hallway.Width, Hallway_SpawnPos.y, Hallway_SpawnPos.z + Hallway.Length);
		Vector3 Hallway_BottomLeft			= new Vector3(Hallway_SpawnPos.x,									Hallway_SpawnPos.y, Hallway_SpawnPos.z);
		Vector3 Hallway_BottomRight			= new Vector3(Hallway_SpawnPos.x + Hallway.Width, Hallway_SpawnPos.y, Hallway_SpawnPos.z);

		//Kitchen_Vertices()
		Vector3 Kitchen_SpawnPos				= new Vector3(DiningRoom_BottomRight.x, 1, DiningRoom_BottomRight.z);

		Vector3 Kitchen_TopLeft					= new Vector3(Kitchen_SpawnPos.x,									 Kitchen_SpawnPos.y, Kitchen_SpawnPos.z + Kitchen.Length);
		Vector3 Kitchen_TopRight				= new Vector3(Kitchen_SpawnPos.x + Kitchen.Width, Kitchen_SpawnPos.y, Kitchen_SpawnPos.z + Kitchen.Length);
		Vector3 Kitchen_BottomLeft			= new Vector3(Kitchen_SpawnPos.x,									 Kitchen_SpawnPos.y, Kitchen_SpawnPos.z);
		Vector3 Kitchen_BottomRight			= new Vector3(Kitchen_SpawnPos.x + Kitchen.Width, Kitchen_SpawnPos.y, Kitchen_SpawnPos.z);


		//Laundry_Vertices()
		Vector3 Laundry_SpawnPos				= new Vector3(Kitchen_BottomRight.x, 1, Kitchen_BottomRight.z);

		Vector3 Laundry_TopLeft					= new Vector3(Laundry_SpawnPos.x,									Laundry_SpawnPos.y, Laundry_SpawnPos.z + Laundry.Length);
		Vector3 Laundry_TopRight				= new Vector3(Laundry_SpawnPos.x + Laundry.Width, Laundry_SpawnPos.y, Laundry_SpawnPos.z + Laundry.Length);
		Vector3 Laundry_BottomLeft			= new Vector3(Laundry_SpawnPos.x,									Laundry_SpawnPos.y, Laundry_SpawnPos.z);
		Vector3 Laundry_BottomRight			= new Vector3(Laundry_SpawnPos.x + Laundry.Width, Laundry_SpawnPos.y, Laundry_SpawnPos.z);

		//EntryWay_Vertices()
		Vector3 EntryWay_SpawnPos				= new Vector3(Laundry_BottomLeft.x, 1, LivingRoom_BottomLeft.z);

		Vector3 EntryWay_TopLeft				= new Vector3(EntryWay_SpawnPos.x,									EntryWay_SpawnPos.y, EntryWay_SpawnPos.z + EntryWay.Length);
		Vector3 EntryWay_TopRight				= new Vector3(EntryWay_SpawnPos.x + EntryWay.Width, EntryWay_SpawnPos.y, EntryWay_SpawnPos.z + EntryWay.Length);
		Vector3 EntryWay_BottomLeft			= new Vector3(EntryWay_SpawnPos.x,									EntryWay_SpawnPos.y, EntryWay_SpawnPos.z);
		Vector3 EntryWay_BottomRight		= new Vector3(EntryWay_SpawnPos.x + EntryWay.Width, EntryWay_SpawnPos.y, EntryWay_SpawnPos.z);




		UpdateSpawnPos("LivingRoom", LivingRoom_SpawnPos);
		UpdateSpawnPos("DiningRoom", DiningRoom_SpawnPos);
		UpdateSpawnPos("Bedroom",		 Bedroom_SpawnPos);
		UpdateSpawnPos("Hallway",		 Hallway_SpawnPos);
		UpdateSpawnPos("Kitchen",		 Kitchen_SpawnPos);
		UpdateSpawnPos("Laundry",		 Laundry_SpawnPos);
	}


	void UpdateSpawnPos(string name, Vector3 newPos)
	{
		var section = RoomIndex.FloorSectionList.Find(s => s.Name == name);
		if (section != null)
			section.SpawnPos = newPos;
	}







	// NOTES

	// EntryWay							— 40.25" × 43"				= 12.01		//sqft
	// EntryCloset					— 35" × 57"						= 13.85		//sqft
	// Hallway							— 36" × 229"					= 57.25		//sqft
	// Bathroom							— 63.75" × 71.25"			= 31.52		//sqft
	// Bedroom							— 157" × 148"					= 161.24	//sqft
	// BedroomCloset				— 78" × 71"					  = 38.42		//sqft
	// Storeroom						— 126" × 200"					= 175.00	//sqft
	// Kitchen							— 90.5" × 114"				= 71.63		//sqft
	// Laundry							— 89.5" × 31.25"			= 19.42		//sqft
	// LivingRoom						— 158.25" × 152"			= 166.96	//sqft
	// DiningRoom						— 95.25" × 125.75"		= 83.23		//sqft
	// Loft									— 137" × 183"					= 174.04	//sqft
	// SecondBedroom				— 130.5" × 165"				= 149.53	//sqft
	// SecondBedroomCloset	— 76" × 23.75"				= 12.53		//sqft
	// TOTAL = 1,166.63 sq ft
	// LVP SQFT = 659.06


	public class Hallway_Measurements
	{
		float length = 36;
		float width = 229;

		public float wall_hallway_south = 36;

		public float wall_hallway_north = 36; // No wall, simply end of hallway as it enters into living room

		public float wall_hallway_east = 229;
			public float wall_hallway_east_kitchen = 48; // 98-40 is the kitchen wall, minus where the living room starts
			public float wall_hallway_east_kitchenEntry_gap = 30;
			public float wall_hallway_east_laundryWall = 41;
			public float wall_hallway_east_entryGap = 43;
			public float wall_hallway_east_waterHeaterWall = 56.75f;

		public float wall_hallway_west = 229;
			public float wall_hallway_west_main = 120;
			public float wall_hallway_west_bedroomDoor_gap = 30;
			public float wall_hallway_west_bathroomDoor_gap = 30;
			public float wall_hallway_west_bedBathWall = 11.5f;
			public float wall_hallway_west_closetWall = 37.5f;
	}
	// ----- ----- ----- ----- ----- ----- ----- Living Room Orginal Measurements ----- ----- ----- ----- ----- ----- ----- 
	public class LivingRoomFullArea_Measurements // Room measured as two rooms. Original one room measurements saved here.
	{
		public float wall_livingroom_north = 253.25f;
			public float wall_livingroom_north_left_east = 58;
			public float wall_livingroom_north_fireplace = 41;
			public float wall_livingroom_north_left_west = 154.25f;

		public float wall_livingroom_south = 253.25f;
			public float wall_livingroom_south_kitchenShelf = 90.5f;
			public float wall_livingroom_south_hallwayPiece = 4.75f;
			public float wall_livingroom_south_hallway_gap = 36;
			public float wall_livingroom_south_mainroom = 122.5f;

		public float wall_livingroom_east = 125.75f;
			public float wall_livingroom_east_stairwell = 28.25f;
			public float wall_livingroom_east_stairFoot = 6;
			public float wall_livingroom_east_underStairs = 91.5f;

		public float wall_livingroom_west = 152;
			public float wall_livingroom_west_left = 28.5f;
			public float wall_livingroom_west_door = 95.5f;
			public float wall_livingroom_west_right = 28;

	}
	// ----- ----- ----- ----- ----- ----- ----- Living Room ----- ----- ----- ----- ----- ----- ----- 
	public class LivingRoom_Measurements
	{
		float length = 158.25f;
		float width = 152;

		public float wall_livingRoom_north = 158.25f;

		public float wall_livingRoom_south = 158.25f;
			public float wall_livingRoom_south_main = 122.25f;
			public float wall_livingRoom_south_hallwayGap = 36;

		public float wall_livingRoom_east = 152;

		public float wall_livingRoom_west = 152;
	}
	// ----- ----- ----- ----- ----- ----- ----- Dining Room ----- ----- ----- ----- ----- ----- ----- 
	public class DiningRoom_Measurements
	{
		float length = 95.25f;
		float width = 125.75f;

		public float wall_diningRoom_north = 95.25f;
			
		public float wall_diningRoom_south = 95.25f;
			public float wall_diningRoom_south_bar = 90.5f;
			public float wall_diningRoom_south_post = 4.75f;

		public float wall_diningRoom_east = 125.75f;
		
		public float wall_diningRoom_west = 125.75f;

	}
	// ----- ----- ----- ----- ----- ----- ----- Entry Way ----- ----- ----- ----- ----- ----- ----- 
	public class EntryWay_Measurements
	{
		float length = 40.25f;
		float width = 43;

		public float wall_entryway_north = 40.25f;

		public float wall_entryway_south = 40.25f;
		public float wall_entryway_south_post = 5.25f;
		public float wall_entryway_south_closet = 30f;
		public float wall_entryway_south_wall = 5;

		public float wall_entryway_east = 43;
		public float wall_entryway_east_doorLeftSide = 3;
		public float wall_entryway_east_doorRightSide = 4;
		public float wall_entryway_frontDoor = 36;

		public float wall_entryway_west_gap = 43;
	}
	// ----- ----- ----- ----- ----- ----- ----- Entry Way Closet ----- ----- ----- ----- ----- ----- ----- 
	public class EntryCloset_Measurements
	{
		float length = 35;
		float width = 57;

		public float wall_entrywayCloset_south = 35;

		public float wall_entrywayCloset_north = 35;
		public float wall_entrywayCloset_north_post = 5;
		public float wall_entrywayCloset_north_gap = 30;

		public float wall_entrywayCloset_east = 57;

		public float wall_entrywayCloset_west = 57;
		public float wall_entrywayCloset_west_inner = 52;
		public float wall_entrywayCloset_west_post = 5;
	}
	// ----- ----- ----- ----- ----- ----- ----- Bathroom ----- ----- ----- ----- ----- ----- ----- 
	public class Bathroom_Measurements
	{
		float length = 63.75f;
		float width = 71.25f;

		public float wall_bathroom_north = 63.75f;
			public float wall_bathroom_north_right = 31.25f;
			public float wall_bathroom_north_doorFrame = 28;
			public float wall_bathroom_north_left = 7;

		public float wall_bathroom_south = 63.75f;

		public float wall_bathroom_east = 71.25f;
			public float wall_bathroom_east_left = 3.25f;
			public float wall_bathroom_east_doorFrame = 30;
			public float wall_bathroom_east_right = 38;

		public float wall_bathroom_west = 71.25f;
			public float wall_bathroom_west_left = 12.25f;
			public float wall_bathroom_west_tub = 58.5f;
	}
	// ----- ----- ----- ----- ----- ----- ----- Bedroom ----- ----- ----- ----- ----- ----- ----- 
	public class Bedroom_Measurements
	{
		float length = 157;
		float width = 148;

		public float wall_bedroom_north = 157;

		public float wall_bedroom_south = 157;
		public float wall_bedroom_south_left = 31.5f;
		public float wall_bedroom_south_bathDoor = 28.25f;
		public float wall_bedroom_south_right = 62f;
		public float wall_bedroom_south_gap = 35.25f;

		public float wall_bedroom_east = 148;
			public float wall_bedroom_east_short = 3;
			public float wall_bedroom_east_doorFrame = 30;
			public float wall_bedroom_east_long = 125;

		public float wall_bedroom_west = 148;
	}
	// ----- ----- ----- ----- ----- ----- ----- BedroomCloset ----- ----- ----- ----- ----- ----- ----- 
	public class BedroomCloset_Measurements
	{
		float length = 78;
		float width = 71;

		public float wall_closet_north = 78;
			public float wall_closet_north_right = 21;
			public float wall_closet_north_gap = 35;
			public float wall_closet_north_left = 22;

		public float wall_closet_south = 78;

		public float wall_closet_east = 71;

		public float wall_closet_west = 71;
			public float wall_closet_west_left = 18;
			public float wall_closet_west_doorFrame = 29.75f;
			public float wall_closet_west_right = 23.12f;
	}
	// ----- ----- ----- ----- ----- ----- ----- Storeroom ----- ----- ----- ----- ----- ----- ----- 
	public class Storeroom_Measurements
	{
		float length = 126;
		float width = 200;

		public float wall_storeroom_north = 126;

		public float wall_storeroom_south = 126f;
			public float wall_storeroom_south_outer = 103.5f;
			public float wall_storeroom_south_inner = 22.5f;

		public float wall_storeroom_east = 200f;

		public float wall_storeroom_west = 200f;
	}
	// ----- ----- ----- ----- ----- ----- ----- Kitchen ----- ----- ----- ----- ----- ----- ----- 
	public class Kitchen_Measurements
	{
		float length = 90.5f;
		float width = 114f;

		public float wall_kitchen_north_kitchenShelf = 90.5f; // where living room wall meets kitchen wall

		public float wall_kitchen_south_hallway = 90.5f; // where laundry space starts

		public float wall_kitchen_east = 114; // where laundry space starts

		public float wall_kitchen_west = 114; // where laundry space starts
	}
	// ----- ----- ----- ----- ----- ----- ----- Laundry Space ----- ----- ----- ----- ----- ----- ----- 
	public class Laundry_Measurements
	{
		float length = 89.5f;
		float width = 31.25f;

		public float wall_laundry_north = 89.5f;

		public float wall_laundry_south = 89.5f;
		public float wall_laundry_south_right = 30f;
		public float wall_laundry_south_gap = 59.5f;

		public float wall_laundry_east = 31.25f;

		public float wall_laundry_west = 31.25f;
	}
	// ----- ----- ----- ----- ----- ----- ----- Loft ----- ----- ----- ----- ----- ----- ----- 
	public class Loft_Measurements
	{
		float length = 137;
		float width = 183;

		public float wall_loft_north = 137;

		public float wall_loft_south = 137;
			public float wall_loft_south_doorEdge = 3.25f;
			public float wall_loft_south_doorGap = 30f;
			public float wall_loft_south_middle = 97.75f;
			public float wall_loft_south_railingEdge = 6;

		public float wall_loft_east = 183;
			public float wall_loft_east_left = 27.5f;
			public float wall_loft_east_railng = 118;
			public float wall_loft_east_right = 37.5f;

		public float wall_loft_west = 183;
			public float wall_loft_west_sidewall = 31.75f;
			public float wall_loft_west_railing = 151.75f;
	}
	// ----- ----- ----- ----- ----- ----- ----- Second Bedroom ----- ----- ----- ----- ----- ----- ----- 
	public class SecondBedroom_Measurements
	{
		float length = 130.5f;
		float width = 165;

		public float wall_north = 130.5f;

		public float wall_south = 130.5f;
			public float wall_south_doorWall = 54.75f;
			public float wall_south_closetWall = 76;

		public float wall_east = 165;
			public float wall_secondBedroom_east_doorWall = 87; // door wall
			public float wall_secondBedroom_east_windowWall = 78.25f; // window wall

		public float wall_west = 165;
	}
	// ----- ----- ----- ----- ----- ----- ----- Second Bedroom Closet ----- ----- ----- ----- ----- ----- ----- 
	public class SecondBedroomCloset_Measurements
	{
		float length = 76;
		float width = 23.75f;

		public float wall_north = 76;
			public float wall_north_right = 15;
			public float wall_north_left = 15;
			public float wall_north_gap = 46;

		public float wall_south = 76;

		public float wall_east = 23.75f;

		public float wall_west = 23.75f;
	}
	// ----- ----- ----- ----- ----- ----- ----- Deck ----- ----- ----- ----- ----- ----- ----- 
	public class Deck_Measurements
	{
		float length = 185f;
		float width = 68.5f;

		public float railing_west = 185f;

		public float railing_north = 68.5f;

		public float wall_door_east = 152.5f;
		public float wall_window_east = 27f;

		public float wall_window_south = 28f;
		public float wall_south = 40f;

	}

	// ----- ----- ----- ----- ----- ----- ----- Second Bedroom Closet Doorframe----- ----- ----- ----- ----- ----- ----- 



}
