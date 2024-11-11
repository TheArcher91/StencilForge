import svgwrite
import cv2
import numpy as np
import random

# Load the image
image_path = "judo1.jpg"  # Change to the path of your image
original_image = cv2.imread(image_path)
if original_image is None:
    raise ValueError("Image not found. Check the image path.")

# Initial settings
zoom_scale = 1.0  # Initial zoom level
dot_radius = 5    # Radius of the red dot
dot_color = (0, 0, 255)  # Red color in BGR
dot_x, dot_y = None, None  # Dot coordinates (center of the rectangle)
dragging = False  # Toggle for dragging the dot
drag_enabled = False  # Controls dot dragging enable/disable
panning = False  # Toggle for panning
offset_x, offset_y = 0, 0  # Offset to simulate zoom center
rectangle_mode = False  # Toggle for rectangle creation
rect_start = None  # Rectangle start position on original image
rect_end = None  # Rectangle end position on original image
random_points = []  # List to store random points
bez_curve_points = []  # List to store points for Bezier curve
random_points_list = []  # List to store random points for each Bezier curve
bezier_curves = []  # List to store all generated Bezier curves
current_function = 0  # Track current function call

# Function to draw the red dot on the image
def draw_dot(image, x, y):
    cv2.circle(image, (int(x), int(y)), dot_radius, dot_color, -1)

# Function to calculate Bezier curve points (using De Casteljau's algorithm)
def bezier_curve(points, num_points=100):
    curve = []
    for t in np.linspace(0, 1, num_points):  # t goes from 0 to 1 in 'num_points' steps
        temp_points = points.copy()  # Copy of control points
        while len(temp_points) > 1:  # Repeat the interpolation until one point remains
            # Linear interpolation between each consecutive pair of points
            temp_points = [((1 - t) * np.array(temp_points[i]) + t * np.array(temp_points[i + 1])).tolist() 
                           for i in range(len(temp_points) - 1)]
        curve.append(temp_points[0])  # The final point in the reduced list is on the curve
    
    # Close the loop by appending the first point to the end
    curve.append(curve[0])  # Ensuring the curve is closed
    
    return np.array(curve)

#Compile Bezier Curves into .svg file
def save_all_beziers_as_svg(bezier_curves, filename="all_bezier_curves.svg"):
    dwg = svgwrite.Drawing(filename, profile="tiny")

    for curve_points in bezier_curves:
        # Check if curve_points is valid and non-empty
        if curve_points is not None and len(curve_points) > 1:
            # Start drawing the path for each curve
            path_data = f"M {curve_points[0][0]},{curve_points[0][1]} "  # Move to the starting point
            for x, y in curve_points[1:]:
                path_data += f"L {x},{y} "  # Draw line to each subsequent point

            # Add the path for each curve to the SVG
            path = dwg.path(d=path_data, stroke=svgwrite.rgb(0, 255, 0, '%'), fill="none", stroke_width=2)
            dwg.add(path)

    dwg.save()
    print(f"All Bezier curves saved as {filename}")

# Function to generate random points inside a rectangle with specified n
def eyebrow_right(rect_start, rect_end, n):
    x_min, y_min = rect_start
    x_max, y_max = rect_end
    x1 = x_min
    y1 = y_min
    x2 = x_max
    y2 = y_max

    a2 = []
    b2 = []
    
    a2.extend([x1 , x1 , x1+(x2-x1)/3 , x1+3*(x2 - x1)/8 , (x1+x2)/2 , x1+2*(x2-x1)/3 , x1+3*(x2-x1)/4 , x2+(x2-x1)/5])
    b2.extend([(y1+y2)/2 , y1 , y2+(y1-y2)/10 , y2+4*(y1-y2)/5 , y2+3*(y1-y2)/4 , y2+2*(y1-y2)/3 , y2+2*(y1-y2)/3 , y2+(y1-y2)/4])
    a2.extend([x2+(x2-x1)/2 , x2-(x2-x1)/5 , (x1+x2)/2 , x1+(x2-x1)/4 , x1+(x2-x1)/5])
    b2.extend([y2-(y1-y2)/2 , y2 , (y1+y2)/2 , (y1+y2)/2 , y2+(y1-y2)/3])

    a2.extend([a2[0]])
    b2.extend([b2[0]])
    
    return [(a2[i], b2[i]) for i in range(len(a2))]
    # points = [
    #     (x_min, (y_min + y_max) / 2), 
    #     (x_min, y_min),
    #     (x_min + (x_max - x_min) / 3, y_min + (y_max - y_min) / 10),
    #     (x_min + 3 * (x_max - x_min) / 8, y_min + 4 * (y_max - y_min) / 5),
    #     ((x_min + x_max) / 2, y_min + 3 * (y_max - y_min) / 4),
    #     (x_min + 2 * (x_max - x_min) / 3, y_min + 2 * (y_max - y_min) / 3),
    #     (x_min + 3 * (x_max - x_min) / 4, y_min + 2 * (y_max - y_min) / 3),
    #     (x_max - (x_max - x_min) / 5, y_max - (y_max - y_min) / 4),
        
    #     # Additional points with linked positions
    #     (x_max - (x_max - x_min) / 2, y_max - (y_max - y_min) / 2),
    #     (x_max - (x_max - x_min) / 5, y_max),
    #     ((x_min + x_max) / 2, (y_min + y_max) / 2),
    #     (x_min + (x_max - x_min) / 4, (y_min + y_max) / 2),
    #     (x_min + (x_max - x_min) / 5, y_max - (y_max - y_min) / 3),
        
    #     # Linking a point to be identical to another point to simulate dragging together
    #     (x_min, y_min)  # This links to the second point in the list
    # ]
    
    # return points

def eyebrow_left(rect_start, rect_end, n):
    x_min, y_min = rect_start
    x_max, y_max = rect_end
    x1 = x_max
    y1 = y_min
    x2 = x_min
    y2 = y_max

    a1 = []
    b1 = []
    
    a1.extend([x1 , x1 , x1+(x2-x1)/3 , x1+3*(x2 - x1)/8 , (x1+x2)/2 , x1+2*(x2-x1)/3 , x1+3*(x2-x1)/4 , x2+(x2-x1)/5])
    b1.extend([(y1+y2)/2 , y1 , y2+(y1-y2)/10 , y2+4*(y1-y2)/5 , y2+3*(y1-y2)/4 , y2+2*(y1-y2)/3 , y2+2*(y1-y2)/3 , y2+(y1-y2)/4])
    a1.extend([x2+(x2-x1)/2 , x2-(x2-x1)/5 , (x1+x2)/2 , x1+(x2-x1)/4 , x1+(x2-x1)/5])
    b1.extend([y2-(y1-y2)/2 , y2 , (y1+y2)/2 , (y1+y2)/2 , y2+(y1-y2)/3])

    a1.extend([a1[0]])
    b1.extend([b1[0]])
    
    return [(a1[i], b1[i]) for i in range(len(a1))]

def eyeball_right(rect_start, rect_end, n):
    x_min, y_min = rect_start
    x_max, y_max = rect_end
    x1 = x_min
    y1 = y_min
    x2 = x_max
    y2 = y_max

    a3 = []
    b3 = []

    a3.extend([x1])
    a3.extend([a3[0] + (x2-x1)/9])
    a3.extend([a3[1] + (x2-x1)/9])
    a3.extend([a3[2] + (x2-x1)/9])
    a3.extend([a3[3] + (x2-x1)/9])
    a3.extend([a3[4] + (x2-x1)/9])
    a3.extend([a3[5] + (x2-x1)/9])
    a3.extend([a3[6] + (x2-x1)/9])
    a3.extend([a3[7] + (x2-x1)/9])
    a3.extend([x2])
    a3.extend([a3[9] - (x2-x1)/9])
    a3.extend([a3[10] - (x2-x1)/9])
    a3.extend([a3[11] - (x2-x1)/9])
    a3.extend([a3[12] - (x2-x1)/9])
    a3.extend([a3[13] - (x2-x1)/9])
    a3.extend([a3[14] - (x2-x1)/9])
    a3.extend([a3[15] - (x2-x1)/9])
    a3.extend([a3[16] - (x2-x1)/9])
    a3.extend([a3[0]])

    b3.extend([y2+2*(y1-y2)/3])
    b3.extend([y1])
    b3.extend([y1])
    b3.extend([y1])
    b3.extend([y1])
    b3.extend([y1])
    b3.extend([y1])
    b3.extend([y1])
    b3.extend([y1])
    b3.extend([(y1+y2)/2])
    b3.extend([y2])
    b3.extend([y2])
    b3.extend([y2])
    b3.extend([y2])
    b3.extend([y2])
    b3.extend([y2])
    b3.extend([y2])
    b3.extend([y2])
    b3.extend([b3[0]])

    return [(a3[i], b3[i]) for i in range(len(a3))]

def eyeball_left(rect_start, rect_end, n):
    x_min, y_min = rect_start
    x_max, y_max = rect_end
    x1 = x_max
    y1 = y_min
    x2 = x_min
    y2 = y_max

    a4 = []
    b4 = []

    a4.extend([x1])
    a4.extend([a4[0] + (x2-x1)/9])
    a4.extend([a4[1] + (x2-x1)/9])
    a4.extend([a4[2] + (x2-x1)/9])
    a4.extend([a4[3] + (x2-x1)/9])
    a4.extend([a4[4] + (x2-x1)/9])
    a4.extend([a4[5] + (x2-x1)/9])
    a4.extend([a4[6] + (x2-x1)/9])
    a4.extend([a4[7] + (x2-x1)/9])
    a4.extend([x2])
    a4.extend([a4[9] - (x2-x1)/9])
    a4.extend([a4[10] - (x2-x1)/9])
    a4.extend([a4[11] - (x2-x1)/9])
    a4.extend([a4[12] - (x2-x1)/9])
    a4.extend([a4[13] - (x2-x1)/9])
    a4.extend([a4[14] - (x2-x1)/9])
    a4.extend([a4[15] - (x2-x1)/9])
    a4.extend([a4[16] - (x2-x1)/9])
    a4.extend([a4[0]])

    b4.extend([y2+2*(y1-y2)/3])
    b4.extend([y1])
    b4.extend([y1])
    b4.extend([y1])
    b4.extend([y1])
    b4.extend([y1])
    b4.extend([y1])
    b4.extend([y1])
    b4.extend([y1])
    b4.extend([(y1+y2)/2])
    b4.extend([y2])
    b4.extend([y2])
    b4.extend([y2])
    b4.extend([y2])
    b4.extend([y2])
    b4.extend([y2])
    b4.extend([y2])
    b4.extend([y2])
    b4.extend([b4[0]])

    return [(a4[i], b4[i]) for i in range(len(a4))]


def cornea_right_1(rect_start, rect_end, n):
    x_min, y_min = rect_start
    x_max, y_max = rect_end
    x1 = x_min
    y1 = y_min
    x2 = x_max
    y2 = y_max

    a5 = []
    b5 = []

    a5.extend([x1+(x2-x1)/4])
    a5.extend([x1+3*(x2-x1)/8])
    a5.extend([x1+5*(x2-x1)/8])
    a5.extend([x1+3*(x2-x1)/4])

    b5.extend([y2+4*(y1-y2)/5])
    b5.extend([y2+4*(y1-y2)/5])
    b5.extend([y2+4*(y1-y2)/5])
    b5.extend([y2+4*(y1-y2)/5])

    return [(a5[i], b5[i]) for i in range(len(a5))]

def cornea_right_2(rect_start, rect_end, n):
    x_min, y_min = rect_start
    x_max, y_max = rect_end
    x1 = x_min
    y1 = y_min
    x2 = x_max
    y2 = y_max

    a6 = []
    b6 = []

    a6.extend([x1+(x2-x1)/4])
    a6.extend([x1+(x2-x1)/4])
    a6.extend([x1+(x2-x1)/4])
    a6.extend([x1+(x2-x1)/4])
    a6.extend([(x1+x2)/2])
    a6.extend([x1+5*(x2-x1)/8])
    a6.extend([x1+5*(x2-x1)/8])
    a6.extend([x1+5*(x2-x1)/8])
    a6.extend([x1+3*(x2-x1)/4])

    b6.extend([y2+4*(y1-y2)/5])
    b6.extend([y2+2*(y1-y2)/3])
    b6.extend([(y1+y2)/2])
    b6.extend([y2+(y1-y2)/3])
    b6.extend([y2+(y1-y2)/3])
    b6.extend([y2+(y1-y2)/3])
    b6.extend([(y1+y2)/2])
    b6.extend([y2+2*(y1-y2)/3])
    b6.extend([y2+4*(y1-y2)/5])


    return [(a6[i], b6[i]) for i in range(len(a6))]


def cornea_left_1(rect_start, rect_end, n):
    x_min, y_min = rect_start
    x_max, y_max = rect_end
    x1 = x_min
    y1 = y_min
    x2 = x_max
    y2 = y_max

    a7 = []
    b7 = []

    a7.extend([x1+(x2-x1)/4])
    a7.extend([x1+3*(x2-x1)/8])
    a7.extend([x1+5*(x2-x1)/8])
    a7.extend([x1+3*(x2-x1)/4])

    b7.extend([y2+4*(y1-y2)/5])
    b7.extend([y2+4*(y1-y2)/5])
    b7.extend([y2+4*(y1-y2)/5])
    b7.extend([y2+4*(y1-y2)/5])

    return [(a7[i], b7[i]) for i in range(len(a7))]

def cornea_left_2(rect_start, rect_end, n):
    x_min, y_min = rect_start
    x_max, y_max = rect_end
    x1 = x_min
    y1 = y_min
    x2 = x_max
    y2 = y_max

    a8 = []
    b8 = []

    a8.extend([x1+(x2-x1)/4])
    a8.extend([x1+(x2-x1)/4])
    a8.extend([x1+(x2-x1)/4])
    a8.extend([x1+(x2-x1)/4])
    a8.extend([(x1+x2)/2])
    a8.extend([x1+5*(x2-x1)/8])
    a8.extend([x1+5*(x2-x1)/8])
    a8.extend([x1+5*(x2-x1)/8])
    a8.extend([x1+3*(x2-x1)/4])

    b8.extend([y2+4*(y1-y2)/5])
    b8.extend([y2+2*(y1-y2)/3])
    b8.extend([(y1+y2)/2])
    b8.extend([y2+(y1-y2)/3])
    b8.extend([y2+(y1-y2)/3])
    b8.extend([y2+(y1-y2)/3])
    b8.extend([(y1+y2)/2])
    b8.extend([y2+2*(y1-y2)/3])
    b8.extend([y2+4*(y1-y2)/5])


    return [(a8[i], b8[i]) for i in range(len(a8))]

def pupil_right(rect_start, rect_end, n):
    x_min, y_min = rect_start
    x_max, y_max = rect_end
    x1 = x_min
    y1 = y_min
    x2 = x_max
    y2 = y_max

    a9 = []
    b9 = []

    a9.extend([(x2+x1)/2])
    a9.extend([a9[0]-(x2-x1)/20])
    a9.extend([a9[1]-(x2-x1)/20])
    a9.extend([(a9[2])])
    a9.extend([a9[3]])
    a9.extend([a9[4]+(x2-x1)/5])
    a9.extend([a9[5]])
    a9.extend([a9[6]])
    a9.extend([a9[7]-(x2-x1)/20])
    a9.extend([a9[0]])

    b9.extend([y2+4*(y1-y2)/5])
    b9.extend([b9[0]-(y1-y2)/12])
    b9.extend([b9[1]-(y1-y2)/12])
    b9.extend([b9[2]-(y1-y2)/12])
    b9.extend([b9[3]-(y1-y2)/12])
    b9.extend([b9[4]])
    b9.extend([b9[5]+(y1-y2)/12])
    b9.extend([b9[6]+(y1-y2)/12])
    b9.extend([b9[7]+(y1-y2)/12])
    b9.extend([b9[0]])

    return [(a9[i], b9[i]) for i in range(len(a9))]



def pupil_left(rect_start, rect_end, n):
    x_min, y_min = rect_start
    x_max, y_max = rect_end
    x1 = x_min
    y1 = y_min
    x2 = x_max
    y2 = y_max

    a10 = []
    b10 = []

    a10.extend([(x2+x1)/2])
    a10.extend([a10[0]-(x2-x1)/20])
    a10.extend([a10[1]-(x2-x1)/20])
    a10.extend([(a10[2])])
    a10.extend([a10[3]])
    a10.extend([a10[4]+(x2-x1)/5])
    a10.extend([a10[5]])
    a10.extend([a10[6]])
    a10.extend([a10[7]-(x2-x1)/20])
    a10.extend([a10[0]])

    b10.extend([y2+4*(y1-y2)/5])
    b10.extend([b10[0]-(y1-y2)/12])
    b10.extend([b10[1]-(y1-y2)/12])
    b10.extend([b10[2]-(y1-y2)/12])
    b10.extend([b10[3]-(y1-y2)/12])
    b10.extend([b10[4]])
    b10.extend([b10[5]+(y1-y2)/12])
    b10.extend([b10[6]+(y1-y2)/12])
    b10.extend([b10[7]+(y1-y2)/12])
    b10.extend([b10[0]])

    return [(a10[i], b10[i]) for i in range(len(a10))]


def nose(rect_start, rect_end, n):
    x_min, y_min = rect_start
    x_max, y_max = rect_end
    x1 = x_min
    y1 = y_min
    x2 = x_max
    y2 = y_max

    a11 = []
    b11 = []

    a11.extend([x1+(x2-x1)/3])
    a11.extend([a11[0]-(x2-x1)/36])
    a11.extend([a11[1]-(x2-x1)/36])
    a11.extend([a11[2]-(x2-x1)/36])
    a11.extend([a11[3]-(x2-x1)/36])
    a11.extend([a11[4]-(x2-x1)/36])
    a11.extend([a11[5]-(x2-x1)/36])
    a11.extend([a11[6]-(x2-x1)/36])
    a11.extend([a11[7]-(x2-x1)/36])
    a11.extend([a11[8]-(x2-x1)/36])
    a11.extend([a11[9]-(x2-x1)/36])
    a11.extend([a11[10]-(x2-x1)/36])
    a11.extend([x1])
    a11.extend([a11[12]+(x2-x1)/12])
    a11.extend([a11[13]+(x2-x1)/12])
    a11.extend([a11[14]+(x2-x1)/12])
    a11.extend([a11[15]+(x2-x1)/12])
    a11.extend([a11[16]+(x2-x1)/12])
    a11.extend([a11[17]+(x2-x1)/12])
    a11.extend([a11[18]+(x2-x1)/12])
    a11.extend([a11[19]+(x2-x1)/12])
    a11.extend([a11[20]+(x2-x1)/12])
    a11.extend([a11[21]+(x2-x1)/12])
    a11.extend([a11[22]+(x2-x1)/12])
    a11.extend([x2])
    a11.extend([a11[24]-(x2-x1)/36])
    a11.extend([a11[25]-(x2-x1)/36])
    a11.extend([a11[26]-(x2-x1)/36])
    a11.extend([a11[27]-(x2-x1)/36])
    a11.extend([a11[28]-(x2-x1)/36])
    a11.extend([a11[29]-(x2-x1)/36])
    a11.extend([a11[30]-(x2-x1)/36])
    a11.extend([a11[31]-(x2-x1)/36])
    a11.extend([a11[32]-(x2-x1)/36])
    a11.extend([a11[33]-(x2-x1)/36])
    a11.extend([a11[34]-(x2-x1)/36])
    a11.extend([a11[35]-(x2-x1)/36])
    a11.extend([x1+3*(x2-x1)/5])
    a11.extend([x1+2*(x2-x1)/5])
    a11.extend([a11[0]])

    b11.extend([y2+4*(y1-y2)/5])
    b11.extend([b11[0]-(y1-y2)/15])
    b11.extend([b11[1]-(y1-y2)/15])
    b11.extend([b11[2]-(y1-y2)/15])
    b11.extend([b11[3]-(y1-y2)/15])
    b11.extend([b11[4]-(y1-y2)/15])
    b11.extend([b11[5]-(y1-y2)/15])
    b11.extend([b11[6]-(y1-y2)/15])
    b11.extend([b11[7]-(y1-y2)/15])
    b11.extend([b11[8]-(y1-y2)/15])
    b11.extend([b11[9]-(y1-y2)/15])
    b11.extend([b11[10]-(y1-y2)/15])
    b11.extend([y2])
    b11.extend([y2])
    b11.extend([y2])
    b11.extend([y2])
    b11.extend([y2])
    b11.extend([y2])
    b11.extend([y2])
    b11.extend([y2])
    b11.extend([y2])
    b11.extend([y2])
    b11.extend([y2])
    b11.extend([y2])
    b11.extend([y2])
    b11.extend([b11[24]+(y1-y2)/15])
    b11.extend([b11[25]+(y1-y2)/15])
    b11.extend([b11[26]+(y1-y2)/15])
    b11.extend([b11[27]+(y1-y2)/15])
    b11.extend([b11[28]+(y1-y2)/15])
    b11.extend([b11[29]+(y1-y2)/15])
    b11.extend([b11[30]+(y1-y2)/15])
    b11.extend([b11[31]+(y1-y2)/15])
    b11.extend([b11[32]+(y1-y2)/15])
    b11.extend([b11[33]+(y1-y2)/15])
    b11.extend([b11[34]+(y1-y2)/15])
    b11.extend([b11[35]+(y1-y2)/15])
    b11.extend([y1])
    b11.extend([y1])
    b11.extend([b11[0]])

    return [(a11[i], b11[i]) for i in range(len(a11))]


def lower_lips_u(rect_start, rect_end, n):
    x_min, y_min = rect_start
    x_max, y_max = rect_end
    x1 = x_min
    y1 = y_min
    x2 = x_max
    y2 = y_max

    a12 = []
    b12 = []

    a12.extend([x1])
    a12.extend([x1+(x2-x1)/20])
    a12.extend([a12[1]+(x2-x1)/10])
    a12.extend([a12[2]+(x2-x1)/10])
    a12.extend([a12[3]+(x2-x1)/10])
    a12.extend([a12[4]+(x2-x1)/10])
    a12.extend([a12[5]+(x2-x1)/10])
    a12.extend([a12[6]+(x2-x1)/10])
    a12.extend([a12[7]+(x2-x1)/10])
    a12.extend([a12[8]+(x2-x1)/10])
    a12.extend([x2-(x2-x1)/20])
    a12.extend([x2])

    b12.extend([y1])
    b12.extend([y2])
    b12.extend([y2])
    b12.extend([y2])
    b12.extend([y2])
    b12.extend([y2])
    b12.extend([y2])
    b12.extend([y2])
    b12.extend([y2])
    b12.extend([y2])
    b12.extend([y2])
    b12.extend([y2])
    b12.extend([y1])

    return [(a12[i], b12[i]) for i in range(len(a12))]

def lower_lips_l(rect_start, rect_end, n):
    x_min, y_min = rect_start
    x_max, y_max = rect_end
    x1 = x_min
    y1 = y_min
    x2 = x_max
    y2 = y_max

    a13 = []
    b13 = []

    a13.extend([x2])
    a13.extend([a13[0]-(x2-x1)/10])
    a13.extend([a13[1]-(x2-x1)/10])
    a13.extend([a13[2]-(x2-x1)/10])
    a13.extend([a13[3]-(x2-x1)/10])
    a13.extend([a13[4]-(x2-x1)/10])
    a13.extend([a13[5]-(x2-x1)/10])
    a13.extend([a13[6]-(x2-x1)/10])
    a13.extend([a13[7]-(x2-x1)/10])
    a13.extend([x1+(x2-x1)/10])
    a13.extend([x1])

    b13.extend([y1])
    b13.extend([y2])
    b13.extend([y2])
    b13.extend([y2])
    b13.extend([y2])
    b13.extend([y2])
    b13.extend([y2])
    b13.extend([y2])
    b13.extend([y2])
    b13.extend([y1])
    b13.extend([y1])

    return [(a13[i], b13[i]) for i in range(len(a13))]


def upper_lips_l(rect_start, rect_end, n):
    x_min, y_min = rect_start
    x_max, y_max = rect_end
    x1 = x_min
    y1 = y_max
    x2 = x_max
    y2 = y_min

    a14 = []
    b14 = []

    a14.extend([x1])
    a14.extend([x1+(x2-x1)/20])
    a14.extend([a14[1]+(x2-x1)/10])
    a14.extend([a14[2]+(x2-x1)/10])
    a14.extend([a14[3]+(x2-x1)/10])
    a14.extend([a14[4]+(x2-x1)/10])
    a14.extend([a14[5]+(x2-x1)/10])
    a14.extend([a14[6]+(x2-x1)/10])
    a14.extend([a14[7]+(x2-x1)/10])
    a14.extend([a14[8]+(x2-x1)/10])
    a14.extend([x2-(x2-x1)/20])
    a14.extend([x2])

    b14.extend([y1])
    b14.extend([y2])
    b14.extend([y2])
    b14.extend([y2])
    b14.extend([y2])
    b14.extend([y2])
    b14.extend([y2])
    b14.extend([y2])
    b14.extend([y2])
    b14.extend([y2])
    b14.extend([y2])
    b14.extend([y2])
    b14.extend([y1])

    return [(a14[i], b14[i]) for i in range(len(a14))]


def upper_lips_u(rect_start, rect_end, n):
    x_min, y_min = rect_start
    x_max, y_max = rect_end
    x1 = x_min
    y1 = y_max
    x2 = x_max
    y2 = y_min

    a15 = []
    b15 = []

    a15.extend([x2])
    a15.extend([a15[0]-(x2-x1)/10])
    a15.extend([a15[1]-(x2-x1)/10])
    a15.extend([a15[2]-(x2-x1)/10])
    a15.extend([a15[3]-(x2-x1)/10])
    a15.extend([a15[4]-(x2-x1)/10])
    a15.extend([a15[5]-(x2-x1)/10])
    a15.extend([a15[6]-(x2-x1)/10])
    a15.extend([a15[7]-(x2-x1)/10])
    a15.extend([x1+(x2-x1)/10])
    a15.extend([x1])

    b15.extend([y1])
    b15.extend([y2])
    b15.extend([y2])
    b15.extend([y2])
    b15.extend([y2])
    b15.extend([y2])
    b15.extend([y2])
    b15.extend([y2])
    b15.extend([y2])
    b15.extend([y1])
    b15.extend([y1])

    return [(a15[i], b15[i]) for i in range(len(a15))]


def facial_inline(rect_start, rect_end, n):
    x_min, y_min = rect_start
    x_max, y_max = rect_end
    x1 = x_min
    y1 = y_max
    x2 = x_max
    y2 = y_min

    a16 = []
    b16 = []

    a16.extend([x1])
    a16.extend([x1])
    a16.extend([x1])
    a16.extend([x1])
    a16.extend([x1])
    a16.extend([x1])
    a16.extend([x1])
    a16.extend([x1])
    a16.extend([x1])
    a16.extend([x1])
    a16.extend([x1])
    a16.extend([x1])
    a16.extend([x1])
    a16.extend([x1])
    a16.extend([x1])
    a16.extend([x1])
    a16.extend([a16[15]+(x2-x1)/15])
    a16.extend([a16[16]+(x2-x1)/15])
    a16.extend([a16[17]+(x2-x1)/15])
    a16.extend([a16[18]+(x2-x1)/15])
    a16.extend([a16[19]+(x2-x1)/15])
    a16.extend([a16[20]+(x2-x1)/15])
    a16.extend([a16[21]+(x2-x1)/15])
    a16.extend([a16[22]+(x2-x1)/15])
    a16.extend([a16[23]+(x2-x1)/15])
    a16.extend([a16[24]+(x2-x1)/15])
    a16.extend([a16[25]+(x2-x1)/15])
    a16.extend([a16[26]+(x2-x1)/15])
    a16.extend([a16[27]+(x2-x1)/15])
    a16.extend([a16[28]+(x2-x1)/15])
    a16.extend([x2])
    a16.extend([x2])
    a16.extend([x2])
    a16.extend([x2])
    a16.extend([x2])
    a16.extend([x2])
    a16.extend([x2])
    a16.extend([x2])
    a16.extend([x2])
    a16.extend([x2])
    a16.extend([x2])
    a16.extend([x2])
    a16.extend([x2])
    a16.extend([x2])
    a16.extend([x2])
    a16.extend([x2])
    a16.extend([a16[45]-(x2-x1)/15])
    a16.extend([a16[46]-(x2-x1)/15])
    a16.extend([a16[47]-(x2-x1)/15])
    a16.extend([a16[48]-(x2-x1)/15])
    a16.extend([a16[49]-(x2-x1)/15])
    a16.extend([a16[50]-(x2-x1)/15])
    a16.extend([a16[51]-(x2-x1)/15])
    a16.extend([a16[52]-(x2-x1)/15])
    a16.extend([a16[53]-(x2-x1)/15])
    a16.extend([a16[54]-(x2-x1)/15])
    a16.extend([a16[55]-(x2-x1)/15])
    a16.extend([a16[56]-(x2-x1)/15])
    a16.extend([a16[57]-(x2-x1)/15])
    a16.extend([a16[58]-(x2-x1)/15])
    a16.extend([a16[0]])

    b16.extend([y1])
    b16.extend([b16[0]-(y1-y2)/15])
    b16.extend([b16[1]-(y1-y2)/15])
    b16.extend([b16[2]-(y1-y2)/15])
    b16.extend([b16[3]-(y1-y2)/15])
    b16.extend([b16[4]-(y1-y2)/15])
    b16.extend([b16[5]-(y1-y2)/15])
    b16.extend([b16[6]-(y1-y2)/15])
    b16.extend([b16[7]-(y1-y2)/15])
    b16.extend([b16[8]-(y1-y2)/15])
    b16.extend([b16[9]-(y1-y2)/15])
    b16.extend([b16[10]-(y1-y2)/15])
    b16.extend([b16[11]-(y1-y2)/15])
    b16.extend([b16[12]-(y1-y2)/15])
    b16.extend([b16[13]-(y1-y2)/15])
    b16.extend([y2])
    b16.extend([y2])
    b16.extend([y2])
    b16.extend([y2])
    b16.extend([y2])
    b16.extend([y2])
    b16.extend([y2])
    b16.extend([y2])
    b16.extend([y2])
    b16.extend([y2])
    b16.extend([y2])
    b16.extend([y2])
    b16.extend([y2])
    b16.extend([y2])
    b16.extend([y2])
    b16.extend([y2])
    b16.extend([b16[30]+(y1-y2)/15])
    b16.extend([b16[31]+(y1-y2)/15])
    b16.extend([b16[32]+(y1-y2)/15])
    b16.extend([b16[33]+(y1-y2)/15])
    b16.extend([b16[34]+(y1-y2)/15])
    b16.extend([b16[35]+(y1-y2)/15])
    b16.extend([b16[36]+(y1-y2)/15])
    b16.extend([b16[37]+(y1-y2)/15])
    b16.extend([b16[38]+(y1-y2)/15])
    b16.extend([b16[39]+(y1-y2)/15])
    b16.extend([b16[40]+(y1-y2)/15])
    b16.extend([b16[41]+(y1-y2)/15])
    b16.extend([b16[42]+(y1-y2)/15])
    b16.extend([b16[43]+(y1-y2)/15])
    b16.extend([y1])
    b16.extend([y1])
    b16.extend([y1])
    b16.extend([y1])
    b16.extend([y1])
    b16.extend([y1])
    b16.extend([y1])
    b16.extend([y1])
    b16.extend([y1])
    b16.extend([y1])
    b16.extend([y1])
    b16.extend([y1])
    b16.extend([y1])
    b16.extend([y1])
    b16.extend([y1])
    b16.extend([b16[0]])

    return [(a16[i], b16[i]) for i in range(len(a16))]


def hair_outline(rect_start, rect_end, n):
    x_min, y_min = rect_start
    x_max, y_max = rect_end
    x1 = x_min
    y1 = y_max
    x2 = x_max
    y2 = y_min

    a17 = []
    b17 = []

    a17.extend([x1])
    a17.extend([x1])
    a17.extend([x1])
    a17.extend([x1])
    a17.extend([x1])
    a17.extend([x1])
    a17.extend([x1])
    a17.extend([x1])
    a17.extend([x1])
    a17.extend([x1])
    a17.extend([x1])
    a17.extend([a17[10]+(x2-x1)/100])
    a17.extend([a17[11]+(x2-x1)/100])
    a17.extend([a17[12]+(x2-x1)/100])
    a17.extend([a17[13]+(x2-x1)/100])
    a17.extend([a17[14]+(x2-x1)/100])
    a17.extend([a17[15]])
    a17.extend([a17[16]])
    a17.extend([a17[17]])
    a17.extend([a17[18]])
    a17.extend([a17[19]])
    a17.extend([a17[20]])
    a17.extend([a17[21]])
    a17.extend([a17[22]])
    a17.extend([a17[23]])
    a17.extend([a17[24]])
    a17.extend([a17[25]])
    a17.extend([a17[26]])
    a17.extend([a17[27]])
    a17.extend([a17[28]])
    a17.extend([a17[29]])
    a17.extend([a17[30]+9*(x2-x1)/200])
    a17.extend([a17[31]+9*(x2-x1)/200])
    a17.extend([a17[32]+9*(x2-x1)/200])
    a17.extend([a17[33]+9*(x2-x1)/200])
    a17.extend([a17[34]+9*(x2-x1)/200])
    a17.extend([a17[35]+9*(x2-x1)/200])
    a17.extend([a17[36]+9*(x2-x1)/200])
    a17.extend([a17[37]+9*(x2-x1)/200])
    a17.extend([a17[38]+9*(x2-x1)/200])
    a17.extend([a17[39]+9*(x2-x1)/200])
    a17.extend([a17[40]+9*(x2-x1)/200])
    a17.extend([a17[41]+9*(x2-x1)/200])
    a17.extend([a17[42]+9*(x2-x1)/200])
    a17.extend([a17[43]+9*(x2-x1)/200])
    a17.extend([a17[44]+9*(x2-x1)/200])
    a17.extend([a17[45]+9*(x2-x1)/200])
    a17.extend([a17[46]+9*(x2-x1)/200])
    a17.extend([a17[47]+9*(x2-x1)/200])
    a17.extend([a17[48]+9*(x2-x1)/200])
    a17.extend([a17[49]+9*(x2-x1)/200])
    a17.extend([a17[50]])
    a17.extend([a17[51]])
    a17.extend([a17[52]])
    a17.extend([a17[53]])
    a17.extend([a17[54]])
    a17.extend([a17[55]])
    a17.extend([a17[56]])
    a17.extend([a17[57]])
    a17.extend([a17[58]])
    a17.extend([a17[59]])
    a17.extend([a17[60]])
    a17.extend([a17[61]])
    a17.extend([a17[62]])
    a17.extend([a17[63]])
    a17.extend([a17[64]])
    a17.extend([a17[65]+(x2-x1)/100])
    a17.extend([a17[66]+(x2-x1)/100])
    a17.extend([a17[67]+(x2-x1)/100])
    a17.extend([a17[68]+(x2-x1)/100])
    a17.extend([a17[69]+(x2-x1)/100])
    a17.extend([x2])
    a17.extend([x2])
    a17.extend([x2])
    a17.extend([x2])
    a17.extend([x2])
    a17.extend([x2])
    a17.extend([x2])
    a17.extend([x2])
    a17.extend([x2])
    a17.extend([x2])
    a17.extend([a17[80]-(x2-x1)/20])
    a17.extend([a17[81]-(x2-x1)/20])
    a17.extend([a17[82]-(x2-x1)/20])
    a17.extend([a17[83]-(x2-x1)/20])
    a17.extend([a17[84]-(x2-x1)/20])
    a17.extend([a17[85]-(x2-x1)/20])
    a17.extend([a17[86]-(x2-x1)/20])
    a17.extend([a17[87]-(x2-x1)/20])
    a17.extend([a17[88]-(x2-x1)/20])
    a17.extend([a17[89]-(x2-x1)/20])
    a17.extend([a17[90]-(x2-x1)/20])
    a17.extend([a17[91]-(x2-x1)/20])
    a17.extend([a17[92]-(x2-x1)/20])
    a17.extend([a17[93]-(x2-x1)/20])
    a17.extend([a17[94]-(x2-x1)/20])
    a17.extend([a17[95]-(x2-x1)/20])
    a17.extend([a17[96]-(x2-x1)/20])
    a17.extend([a17[97]-(x2-x1)/20])
    a17.extend([a17[98]-(x2-x1)/20])
    a17.extend([a17[0]])

    b17.extend([y1])
    b17.extend([y1-(y1-y2)/10])
    b17.extend([b17[1]-(y1-y2)/10])
    b17.extend([b17[2]-(y1-y2)/10])
    b17.extend([b17[3]-(y1-y2)/10])
    b17.extend([b17[4]-(y1-y2)/10])
    b17.extend([b17[5]-(y1-y2)/10])
    b17.extend([b17[6]-(y1-y2)/10])
    b17.extend([b17[7]-(y1-y2)/10])
    b17.extend([b17[8]-(y1-y2)/10])
    b17.extend([y2])
    b17.extend([y2])
    b17.extend([y2])
    b17.extend([y2])
    b17.extend([y2])
    b17.extend([y2])
    b17.extend([y2+4*(y1-y2)/75])
    b17.extend([b17[16]+4*(y1-y2)/75])
    b17.extend([b17[17]+4*(y1-y2)/75])
    b17.extend([b17[18]+4*(y1-y2)/75])
    b17.extend([b17[19]+4*(y1-y2)/75])
    b17.extend([b17[20]+4*(y1-y2)/75])
    b17.extend([b17[21]+4*(y1-y2)/75])
    b17.extend([b17[22]+4*(y1-y2)/75])
    b17.extend([b17[23]+4*(y1-y2)/75])
    b17.extend([b17[24]+4*(y1-y2)/75])
    b17.extend([b17[25]+4*(y1-y2)/75])
    b17.extend([b17[26]+4*(y1-y2)/75])
    b17.extend([b17[27]+4*(y1-y2)/75])
    b17.extend([b17[28]+4*(y1-y2)/75])
    b17.extend([b17[29]+4*(y1-y2)/75])
    b17.extend([b17[30]])
    b17.extend([b17[31]])
    b17.extend([b17[32]])
    b17.extend([b17[33]])
    b17.extend([b17[34]])
    b17.extend([b17[35]])
    b17.extend([b17[36]])
    b17.extend([b17[37]])
    b17.extend([b17[38]])
    b17.extend([b17[39]])
    b17.extend([b17[40]])
    b17.extend([b17[41]])
    b17.extend([b17[42]])
    b17.extend([b17[43]])
    b17.extend([b17[44]])
    b17.extend([b17[45]])
    b17.extend([b17[46]])
    b17.extend([b17[47]])
    b17.extend([b17[48]])
    b17.extend([b17[49]])
    b17.extend([b17[50]-4*(y1-y2)/75])
    b17.extend([b17[51]-4*(y1-y2)/75])
    b17.extend([b17[52]-4*(y1-y2)/75])
    b17.extend([b17[53]-4*(y1-y2)/75])
    b17.extend([b17[54]-4*(y1-y2)/75])
    b17.extend([b17[55]-4*(y1-y2)/75])
    b17.extend([b17[56]-4*(y1-y2)/75])
    b17.extend([b17[57]-4*(y1-y2)/75])
    b17.extend([b17[58]-4*(y1-y2)/75])
    b17.extend([b17[59]-4*(y1-y2)/75])
    b17.extend([b17[60]-4*(y1-y2)/75])
    b17.extend([b17[61]-4*(y1-y2)/75])
    b17.extend([b17[62]-4*(y1-y2)/75])
    b17.extend([b17[63]-4*(y1-y2)/75])
    b17.extend([y2])
    b17.extend([y2])
    b17.extend([y2])
    b17.extend([y2])
    b17.extend([y2])
    b17.extend([y2])
    b17.extend([b17[70]+(y1-y2)/10])
    b17.extend([b17[71]+(y1-y2)/10])
    b17.extend([b17[72]+(y1-y2)/10])
    b17.extend([b17[73]+(y1-y2)/10])
    b17.extend([b17[74]+(y1-y2)/10])
    b17.extend([b17[75]+(y1-y2)/10])
    b17.extend([b17[76]+(y1-y2)/10])
    b17.extend([b17[77]+(y1-y2)/10])
    b17.extend([b17[78]+(y1-y2)/10])
    b17.extend([y1])
    b17.extend([y1])
    b17.extend([y1])
    b17.extend([y1])
    b17.extend([y1])
    b17.extend([y1])
    b17.extend([y1])
    b17.extend([y1])
    b17.extend([y1])
    b17.extend([y1])
    b17.extend([y1])
    b17.extend([y1])
    b17.extend([y1])
    b17.extend([y1])
    b17.extend([y1])
    b17.extend([y1])
    b17.extend([y1])
    b17.extend([y1])
    b17.extend([y1])
    b17.extend([y1])
    b17.extend([b17[0]])

    return [(a17[i], b17[i]) for i in range(len(a17))]



def ear_right(rect_start, rect_end, n):
    x_min, y_min = rect_start
    x_max, y_max = rect_end
    x1 = x_min
    y1 = y_max
    x2 = x_max
    y2 = y_min

    a18 = []
    b18 = []

    a18.extend([x2])
    a18.extend([a18[0]-(x2-x1)/4])
    a18.extend([a18[1]-(x2-x1)/4])
    a18.extend([a18[2]-(x2-x1)/4])
    a18.extend([x1])
    a18.extend([x1])
    a18.extend([x1])
    a18.extend([x1])
    a18.extend([x1])
    a18.extend([x1])
    a18.extend([x1])
    a18.extend([x1])
    a18.extend([x1])
    a18.extend([x1])
    a18.extend([x1])
    a18.extend([a18[14]+(x2-x1)/4])
    a18.extend([a18[15]+(x2-x1)/4])
    a18.extend([a18[16]+(x2-x1)/4])
    a18.extend([x2])
    a18.extend([x2])
    a18.extend([x2])
    a18.extend([x2])
    a18.extend([x2])
    a18.extend([x2])
    a18.extend([x2])
    a18.extend([x2])
    a18.extend([x2])
    a18.extend([x2])
    a18.extend([a18[0]])

    b18.extend([y1])
    b18.extend([y1])
    b18.extend([y1])
    b18.extend([y1])
    b18.extend([y1])
    b18.extend([b18[4]-(y1-y2)/10])
    b18.extend([b18[5]-(y1-y2)/10])
    b18.extend([b18[6]-(y1-y2)/10])
    b18.extend([b18[7]-(y1-y2)/10])
    b18.extend([b18[8]-(y1-y2)/10])
    b18.extend([b18[9]-(y1-y2)/10])
    b18.extend([b18[10]-(y1-y2)/10])
    b18.extend([b18[11]-(y1-y2)/10])
    b18.extend([b18[12]-(y1-y2)/10])
    b18.extend([y2])
    b18.extend([y2])
    b18.extend([y2])
    b18.extend([y2])
    b18.extend([y2])
    b18.extend([b18[18]+(y1-y2)/10])
    b18.extend([b18[19]+(y1-y2)/10])
    b18.extend([b18[20]+(y1-y2)/10])
    b18.extend([b18[21]+(y1-y2)/10])
    b18.extend([b18[22]+(y1-y2)/10])
    b18.extend([b18[23]+(y1-y2)/10])
    b18.extend([b18[24]+(y1-y2)/10])
    b18.extend([b18[25]+(y1-y2)/10])
    b18.extend([b18[26]+(y1-y2)/10])
    b18.extend([b18[0]])

    return [(a18[i], b18[i]) for i in range(len(a18))]
    


def ear_left(rect_start, rect_end, n):
    x_min, y_min = rect_start
    x_max, y_max = rect_end
    x1 = x_min
    y1 = y_max
    x2 = x_max
    y2 = y_min

    a19 = []
    b19 = []

    a19.extend([x2])
    a19.extend([a19[0]-(x2-x1)/4])
    a19.extend([a19[1]-(x2-x1)/4])
    a19.extend([a19[2]-(x2-x1)/4])
    a19.extend([x1])
    a19.extend([x1])
    a19.extend([x1])
    a19.extend([x1])
    a19.extend([x1])
    a19.extend([x1])
    a19.extend([x1])
    a19.extend([x1])
    a19.extend([x1])
    a19.extend([x1])
    a19.extend([x1])
    a19.extend([a19[14]+(x2-x1)/4])
    a19.extend([a19[15]+(x2-x1)/4])
    a19.extend([a19[16]+(x2-x1)/4])
    a19.extend([x2])
    a19.extend([x2])
    a19.extend([x2])
    a19.extend([x2])
    a19.extend([x2])
    a19.extend([x2])
    a19.extend([x2])
    a19.extend([x2])
    a19.extend([x2])
    a19.extend([x2])
    a19.extend([a19[0]])

    b19.extend([y1])
    b19.extend([y1])
    b19.extend([y1])
    b19.extend([y1])
    b19.extend([y1])
    b19.extend([b19[4]-(y1-y2)/10])
    b19.extend([b19[5]-(y1-y2)/10])
    b19.extend([b19[6]-(y1-y2)/10])
    b19.extend([b19[7]-(y1-y2)/10])
    b19.extend([b19[8]-(y1-y2)/10])
    b19.extend([b19[9]-(y1-y2)/10])
    b19.extend([b19[10]-(y1-y2)/10])
    b19.extend([b19[11]-(y1-y2)/10])
    b19.extend([b19[12]-(y1-y2)/10])
    b19.extend([y2])
    b19.extend([y2])
    b19.extend([y2])
    b19.extend([y2])
    b19.extend([y2])
    b19.extend([b19[18]+(y1-y2)/10])
    b19.extend([b19[19]+(y1-y2)/10])
    b19.extend([b19[20]+(y1-y2)/10])
    b19.extend([b19[21]+(y1-y2)/10])
    b19.extend([b19[22]+(y1-y2)/10])
    b19.extend([b19[23]+(y1-y2)/10])
    b19.extend([b19[24]+(y1-y2)/10])
    b19.extend([b19[25]+(y1-y2)/10])
    b19.extend([b19[26]+(y1-y2)/10])
    b19.extend([b19[0]])

    return [(a19[i], b19[i]) for i in range(len(a19))]


def neck1(rect_start, rect_end, n):
    x_min, y_min = rect_start
    x_max, y_max = rect_end
    x1 = x_min
    y1 = y_max
    x2 = x_max
    y2 = y_min

    a20 = []
    b20 = []

    a20.extend([x1+(x2-x1)/3])
    a20.extend([a20[0]])
    a20.extend([a20[1]])
    a20.extend([a20[2]])
    a20.extend([a20[3]])
    a20.extend([a20[4]])
    a20.extend([a20[5]-(x2-x1)/18])
    a20.extend([a20[6]-(x2-x1)/18])
    a20.extend([a20[7]-(x2-x1)/18])
    a20.extend([a20[8]-(x2-x1)/18])
    a20.extend([a20[9]-(x2-x1)/18])
    a20.extend([a20[10]-(x2-x1)/18])

    b20.extend([y1])
    b20.extend([b20[0]-(y1-y2)/30])
    b20.extend([b20[1]-(y1-y2)/30])
    b20.extend([b20[2]-(y1-y2)/30])
    b20.extend([b20[3]-(y1-y2)/30])
    b20.extend([b20[4]-(y1-y2)/30])
    b20.extend([b20[5]-(y1-y2)/12])
    b20.extend([b20[6]-(y1-y2)/12])
    b20.extend([b20[7]-(y1-y2)/12])
    b20.extend([b20[8]-(y1-y2)/12])
    b20.extend([b20[9]-(y1-y2)/12])
    b20.extend([b20[10]-(y1-y2)/12])


    return [(a20[i], b20[i]) for i in range(len(a20))]


def neck2(rect_start, rect_end, n):
    x_min, y_min = rect_start
    x_max, y_max = rect_end
    x1 = x_min
    y1 = y_max
    x2 = x_max
    y2 = y_min

    a21 = []
    b21 = []

    a21.extend([x1])
    a21.extend([x1])

    b21.extend([y1])
    b21.extend([y2])

    return [(a21[i], b21[i]) for i in range(len(a21))]


def neck3(rect_start, rect_end, n):
    x_min, y_min = rect_start
    x_max, y_max = rect_end
    x1 = x_min
    y1 = y_max
    x2 = x_max
    y2 = y_min

    a22 = []
    b22 = []

    a22.extend([x1])
    a22.extend([x2])

    b22.extend([y1])
    b22.extend([y1])

    return [(a22[i], b22[i]) for i in range(len(a22))]

def neck4(rect_start, rect_end, n):
    x_min, y_min = rect_start
    x_max, y_max = rect_end
    x1 = x_min
    y1 = y_max
    x2 = x_max
    y2 = y_min

    a23 = []
    b23 = []

    a23.extend([x2])
    a23.extend([x2])

    b23.extend([y1])
    b23.extend([y2])

    return [(a23[i], b23[i]) for i in range(len(a23))]


def neck5(rect_start, rect_end, n):
    x_min, y_min = rect_start
    x_max, y_max = rect_end
    x1 = x_max
    y1 = y_max
    x2 = x_min
    y2 = y_min

    a24 = []
    b24 = []

    a24.extend([x1+(x2-x1)/3])
    a24.extend([a24[0]])
    a24.extend([a24[1]])
    a24.extend([a24[2]])
    a24.extend([a24[3]])
    a24.extend([a24[4]])
    a24.extend([a24[5]-(x2-x1)/18])
    a24.extend([a24[6]-(x2-x1)/18])
    a24.extend([a24[7]-(x2-x1)/18])
    a24.extend([a24[8]-(x2-x1)/18])
    a24.extend([a24[9]-(x2-x1)/18])
    a24.extend([a24[10]-(x2-x1)/18])

    b24.extend([y1])
    b24.extend([b24[0]-(y1-y2)/30])
    b24.extend([b24[1]-(y1-y2)/30])
    b24.extend([b24[2]-(y1-y2)/30])
    b24.extend([b24[3]-(y1-y2)/30])
    b24.extend([b24[4]-(y1-y2)/30])
    b24.extend([b24[5]-(y1-y2)/12])
    b24.extend([b24[6]-(y1-y2)/12])
    b24.extend([b24[7]-(y1-y2)/12])
    b24.extend([b24[8]-(y1-y2)/12])
    b24.extend([b24[9]-(y1-y2)/12])
    b24.extend([b24[10]-(y1-y2)/12])


    return [(a24[i], b24[i]) for i in range(len(a24))]

def neck6(rect_start, rect_end, n):
    x_min, y_min = rect_start
    x_max, y_max = rect_end
    x1 = x_max
    y1 = y_max
    x2 = x_min
    y2 = y_min

    a25 = []
    b25 = []

    a25.extend([x2])
    a25.extend([x1])

    b25.extend([y1])
    b25.extend([y1])

    return [(a25[i], b25[i]) for i in range(len(a25))]



# 10 functions for different n values
def function1(): return eyebrow_right(rect_start, rect_end, 5)
def function2(): return eyebrow_left(rect_start, rect_end, 11)
def function3(): return eyeball_right(rect_start, rect_end, 12)
def function4(): return eyeball_left(rect_start, rect_end, 13)
def function5(): return cornea_right_1(rect_start, rect_end, 14)
def function6(): return cornea_right_2(rect_start, rect_end, 15)
def function7(): return cornea_left_1(rect_start, rect_end, 16)
def function8(): return cornea_left_2(rect_start, rect_end, 17)
def function9(): return pupil_right(rect_start, rect_end, 18)
def function10(): return pupil_left(rect_start, rect_end, 19)
def function11(): return nose(rect_start, rect_end, 20)
def function12(): return lower_lips_u(rect_start, rect_end, 21)
def function13(): return lower_lips_l(rect_start, rect_end, 22)
def function14(): return upper_lips_l(rect_start, rect_end, 23)
def function15(): return upper_lips_u(rect_start, rect_end, 24)
def function16(): return facial_inline(rect_start, rect_end, 25)
def function17(): return hair_outline(rect_start, rect_end, 26)
def function18(): return ear_right(rect_start, rect_end, 27)
def function19(): return ear_left(rect_start, rect_end, 28)
def function20(): return neck1(rect_start, rect_end, 29)
def function21(): return neck2(rect_start, rect_end, 9)
def function22(): return neck3(rect_start, rect_end, 9)
def function23(): return neck4(rect_start, rect_end, 9)
def function24(): return neck5(rect_start, rect_end, 9)
def function25(): return neck6(rect_start, rect_end, 9)

# List of functions to call sequentially
random_point_functions = [function1, function2, function3, function4, function5, 
                          function6, function7, function8, function9, function10,
                          function11, function12, function13, function14, function15,
                          function16, function17, function18, function19, function20,
                          function21, function22, function23, function24, function25]

# Mouse callback function
def mouse_callback(event, x, y, flags, param):
    global dot_x, dot_y, dragging, zoom_scale, offset_x, offset_y
    global panning, last_mouse_x, last_mouse_y, rectangle_mode, rect_start, rect_end
    global random_points, bez_curve_points, random_points_list, bezier_curves, current_function

    # Rectangle creation on right mouse drag
    if rectangle_mode and event == cv2.EVENT_RBUTTONDOWN:
        # Record the starting point of the rectangle in terms of the original image coordinates
        rect_start = ((x - offset_x) / zoom_scale, (y - offset_y) / zoom_scale)
        rect_end = None

    elif rectangle_mode and event == cv2.EVENT_MOUSEMOVE and rect_start:
        if flags & cv2.EVENT_FLAG_RBUTTON:
            # Update the end point of the rectangle as the mouse moves
            rect_end = ((x - offset_x) / zoom_scale, (y - offset_y) / zoom_scale)

    elif rectangle_mode and event == cv2.EVENT_RBUTTONUP and rect_start:
        # Finalize the end point of the rectangle
        rect_end = ((x - offset_x) / zoom_scale, (y - offset_y) / zoom_scale)

        # Check if we have more functions to call in the sequence
        if current_function < len(random_point_functions):
            # Call the current function to generate random points
            random_points = random_point_functions[current_function]()
            random_points_list.append(random_points)  # Store points in the list
            bez_curve_points = random_points.copy()   # Prepare for Bezier generation

            # Generate and store the corresponding Bezier curve
            bez_curve = bezier_curve(random_points)
            bezier_curves.append(bez_curve)

            # Increment to the next function for the next 'R' press
            current_function += 1

    # Enable dragging on left mouse click for grid points
    if drag_enabled and event == cv2.EVENT_LBUTTONDOWN:
        for i, (px, py) in enumerate(random_points):
            if abs(x - (px * zoom_scale + offset_x)) <= dot_radius and abs(y - (py * zoom_scale + offset_y)) <= dot_radius:
                dragging = i
                # Clear the old Bezier curve to prepare for the new one
                bez_curve_points = random_points.copy()

    elif drag_enabled and event == cv2.EVENT_MOUSEMOVE and dragging is not False:
        # Update the point as it is being dragged
        random_points[dragging] = (x - offset_x) / zoom_scale, (y - offset_y) / zoom_scale

        # Update the Bezier curve in real-time with the new positions
        bez_curve = bezier_curve(random_points)
        bezier_curves[-1] = bez_curve  # Update the latest Bezier curve


    elif drag_enabled and event == cv2.EVENT_LBUTTONUP:
        # When dragging is done, update the Bezier curve based on the final positions
        bez_curve_points = random_points.copy()  # Finalize the Bezier curve with updated points
        bez_curve = bezier_curve(random_points)  # Regenerate the Bezier curve with updated points
        bezier_curves[-1] = bez_curve  # Update the last Bezier curve in the list
        dragging = False


    # Panning with middle mouse button
    elif event == cv2.EVENT_MBUTTONDOWN:
        panning = True
        last_mouse_x, last_mouse_y = x, y

    elif event == cv2.EVENT_MOUSEMOVE and panning:
        dx, dy = x - last_mouse_x, y - last_mouse_y
        offset_x += dx
        offset_y += dy
        last_mouse_x, last_mouse_y = x, y

    elif event == cv2.EVENT_MBUTTONUP:
        panning = False

    # Zoom with Ctrl + Scroll
    if event == cv2.EVENT_MOUSEWHEEL and (flags & cv2.EVENT_FLAG_CTRLKEY):
        old_zoom_scale = zoom_scale
        zoom_scale *= 1.1 if flags > 0 else 0.9
        if zoom_scale < 0.1:
            zoom_scale = 0.1

        offset_x += int(x * (1 - zoom_scale / old_zoom_scale))
        offset_y += int(y * (1 - zoom_scale / old_zoom_scale))

# Main loop
cv2.namedWindow("Zoomed Image")
cv2.setMouseCallback("Zoomed Image", mouse_callback)

while True:
    # Resize the image for zooming
    zoomed_image = cv2.resize(
        original_image,
        None,
        fx=zoom_scale,
        fy=zoom_scale,
        interpolation=cv2.INTER_LINEAR
    )

    # Apply the offset to simulate zoom center at mouse position
    height, width = zoomed_image.shape[:2]
    display_image = np.zeros_like(zoomed_image)
    y1, y2 = max(0, int(offset_y)), min(height, int(height + offset_y))
    x1, x2 = max(0, int(offset_x)), min(width, int(width + offset_x))
    display_image[y1:y2, x1:x2] = zoomed_image[
        y1 - int(offset_y):y2 - int(offset_y),
        x1 - int(offset_x):x2 - int(offset_x)
    ]

    # Draw the rectangle if defined, adjusting for zoom and offset
    if rect_start and rect_end:
        start_x = int(rect_start[0] * zoom_scale + offset_x)
        start_y = int(rect_start[1] * zoom_scale + offset_y)
        end_x = int(rect_end[0] * zoom_scale + offset_x)
        end_y = int(rect_end[1] * zoom_scale + offset_y)
        cv2.rectangle(display_image, (start_x, start_y), (end_x, end_y), (0, 0, 0), 2)  # Black border

    # Draw each set of random points
    for points in random_points_list:
        for px, py in points:
            draw_dot(display_image, px * zoom_scale + offset_x, py * zoom_scale + offset_y)

    # Draw each stored Bezier curve
    for bez_points in bezier_curves:
        for i in range(len(bez_points) - 1):
            # Apply zoom and offset to each point in the Bezier curve
            start_x = int(bez_points[i][0] * zoom_scale + offset_x)
            start_y = int(bez_points[i][1] * zoom_scale + offset_y)
            end_x = int(bez_points[i + 1][0] * zoom_scale + offset_x)
            end_y = int(bez_points[i + 1][1] * zoom_scale + offset_y)

            # Draw the Bezier curve on the display image
            cv2.line(display_image, (start_x, start_y), (end_x, end_y), (0, 255, 0), 2)  # Green curve

    # Show the image
    cv2.imshow("Zoomed Image", display_image)

    # Handle key press events
    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        # Exit the loop on 'q' and output the final coordinates
        if dot_x is not None and dot_y is not None:
            final_x = dot_x
            final_y = original_image.shape[0] - dot_y
            print(f"Final coordinates (relative to bottom-left): ({final_x:.2f}, {final_y:.2f})")

        # Save all Bezier curves to an SVG file
        all_original_bez_points = [bezier_curve(points) for points in random_points_list]
        save_all_beziers_as_svg(all_original_bez_points)  # Save all curves to SVG
        break

    elif key == ord('r'):
        # Enable rectangle mode on 'R'
        rectangle_mode = True

    elif key == ord('u'):
        # Enable/disable dragging for points
        drag_enabled = not drag_enabled

cv2.destroyAllWindows()
