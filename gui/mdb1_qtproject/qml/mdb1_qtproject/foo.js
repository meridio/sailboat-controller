
function distance(boat_lat,boat_lon,taget_lat,target_lon)
{
    x++; // The value of x is incremented by 1.
 
    // Following line will return the text as below.
    return "<h2>Your click count is : "+x+"</h2>";
}


function target_heading(boat_lat,boat_lon,target_lat,target_lon)
{
    var x,y,res;
	x=boat_lat-target_lat;
	y=boat_lon-target_lon;

	res=atan(y/x);

    return "FOO:  X="+x+", Y="+y+", ATAN:"+res;
}
