# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- rigid2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. We need to be able to _normalize_ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the _normalize_ functionality  
        i. Add a member function to the Vector2D struct for normalizing.  
        ii. Write an external function that takes in a Vector2D and returns a normalized copy.  
        iii. Overload the () operator for Vector2D objects to normalize the Vector2D that calls it.  

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.  
        i.      Pros:  
                Groups the function with the object it is related to.  
                Cons:  
                Normalized vector would not update if the user changed x and y values of the Vector2D unless that function were called again. This may not be intuitive to people using this struct. Guideline C.4 states a function should be a member only if it needs direct access to the representation. Since struct members are public, this isn't necessary.  

        ii.     Pros:  
                Much more intuitive and does not overwrite the original Vector2D that was passed into it. Not treated like an attribute of the data structure (guideline C.4).  
                Cons: Not grouped with the struct itself (though this ends up being a good thing as well).  

        iii.    Pros:  
                Shorthand way of calling the functionality of normalizing.  
                Cons:  
                Not intuitive at all and overloads an operator in a way that doesn't match its conventional meaning, going against guideline C.167.  


   - Which of the methods would you implement and why?  
        I implemented the second design and wrote an external function to return a normalized version of the Vector2D passed into it. This method seems to meet the guidelines the most and provides the least ambiguity in its usage.

2. What is the difference between a class and a struct in C++?  
    A class and a struct are very similar in C++, however the difference is that members of a struct are public by default, 
    whereas classes have private members by default.  

3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?  
    The members of Vector2D are public and we are okay with them being easily accessible, while in Transform2D we don't want to 
    allow public access to the member variables (Guidelines C.8 and C.9). This is because a Vector2D does not have an invariant, whereas 
    in Transform2D there may be private member variables where an invariant should be enforced.  

4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?  
    C++ core guideline C.46 states that single-argument constructors should be declared explicit to avoid unintended conversions.  

5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer  
    C++ code guideline Con.2 states that by default we should make member functions const. This is because we don't want to change the object itself when we call the member function, in the case of Transform2D::inv(). When overwriting the \*= operator for Transform2D objects, however, we want to change the object itself as a result of calling this function. T \*= T1 is the same as T = T\*T1, so we are overwriting T here. Therefore the result of calling \*= does not keep the object constant, so we should not declare it as such.