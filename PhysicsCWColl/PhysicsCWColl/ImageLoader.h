/*
// A basic Image loader class used to load in textures.
// Author: 
// Jordan S. Gray
// 40087220
// Edinburgh Napier University
// BSc Games Development
// Year 3
// graybostephano@gmail.com
//
// Credit to:
// www.stackoverflow.com for samples of image loading class.
*/

#ifndef IMAGE_LOADER_H_INCLUDED
#define IMAGE_LOADER_H_INCLUDED

//Image class holds image data fo any included textures.
class Image
{
public:
	Image(char* ps, int w, int h);	//Image constructor takes in char pixels, int width, int height
	~Image();

	/*
	// An array indicating colour of each pixel in image.
	// Color ranges from 0 - 255.
	// Array starts at bottom left then moves right to end of row then onto next column.
	*/
	char* pixels;
	int width;
	int height;
	
};

//Reads a bitmap image from file.
Image* loadBMP(const char* filename);

#endif

