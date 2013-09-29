
/*
 * 	POINT (x,y) and LINE (Ax+By=C) type definitons
 */

typedef struct {
  double x, y;
} Point;

typedef struct {
  double A, B, C;
} Line;



/*
 *	Custom compare function to sort waypoints objects by Y ascending
 */
int cmpfunc(const void * a, const void * b)
{
	Point *ia = (Point *)a;
	Point *ib = (Point *)b;
	return (int)(ia->y - ib->y);
}

/*
 *	Create a new Point object
 */
Point new_point(double x, double y)
{
	Point p;
	p.x = x;
	p.y = y;
	return p;
}

/*
 *	Rotate point coordinates
 */
Point rotate_point(Point p, float angle)
{
	Point t;
	double alpha = (angle+180)*(PI/180.0);
	t.x = p.x*cosf(alpha) - p.y*sinf(alpha);
	t.y = p.x*sinf(alpha) + p.y*cosf(alpha);
	return t;
}

/*
 *	Convert point coordinates to XY
 */
Point convert_xy(Point p)
{
	Point t;
	t.x = p.x*CONVLON;
	t.y = p.y*CONVLAT;
	return t;
}

/*
 *	Convert point coordinates to LAT-LON
 */
Point convert_latlon(Point p)
{
	Point t;
	t.x = p.x/CONVLON;
	t.y = p.y/CONVLAT;
	return t;
}

/*
 *	Create a new Line object as Ax+By=C
 */
Line new_line(Point p1, Point p2)
{
	Line l;
	l.A=p2.y-p1.y;
	l.B=p1.x-p2.x;
	l.C=l.A*p1.x+l.B*p1.y;
	return l;
}

/*
 *	Find the intersection point between two lines
 */
Point find_intersection(Line l1, Line l2)
{
	Point t;	
	double det = l1.A*l2.B - l2.A*l1.B;
	if(det == 0){
		//Lines are parallel
		t.x=0; t.y=0;
	}else{
		t.x = (l2.B*l1.C - l1.B*l2.C)/det;
		t.y = (l1.A*l2.C - l2.A*l1.C)/det;
	}
	return t;
}

/*
 *	Calculate the distance between two points in the XY coordinates, return meters
 */
double calculate_distance_xy(Point p1, Point p2)
{

	double distance;
	double distance_x = p1.x-p2.x;
	double distance_y = p1.y-p2.y; 

	distance = sqrt( (distance_x * distance_x) + (distance_y * distance_y));

	return distance;
}
