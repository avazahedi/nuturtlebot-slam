# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- rigid2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. We need to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality
        i. 

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.

   - Which of the methods would you implement and why?

2. What is the difference between a class and a struct in C++?
    A class and a struct are very similar in C++, however the main difference is that members of a struct are public by default, 
    whereas classes have private members by default.

3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?
    The members of Vector2D are public and we are okay with them being easily accessible, while in Transform2D we don't want to 
    allow public access to the member variables (Guidelines C.8 and C.9).


4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?


5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer

