/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/* 
 * Base code for CS 296 Software Systems Lab 
 * Department of Computer Science and Engineering, IIT Bombay
 * Instructor: Parag Chaudhuri
 */


#ifndef _CS296BASE_HPP_
#define _CS296BASE_HPP_

#include "render.hpp"
#include <Box2D/Box2D.h>
#include <cstdlib>

#define	RAND_LIMIT 32767

namespace cs296
{

  //! The basic difference between class and struct is that default members in class are private while in struct are public.
  //! You can change this for a class by specifying public or private but not for struct.
  class base_sim_t;
  struct settings_t;
  
  //! Why do we use a typedef
  //! typedef is used in C and C++ to denote more complex data types with simple names.
  typedef base_sim_t* sim_create_fcn(); 

  //! Simulation settings. Some can be controlled in the GUI.
  struct settings_t
  {
    //! How is this happening?
    //! Data types of the class members are defined later. 
    //! Constructor of all the variables are called with given arguments and thus variables are defined.
    //! Notice the initialization of the class members in the constructor
    //! All distinct variables are seperated by commas
    settings_t() :
      view_center(0.0f, 20.0f), /** A b2Vec2 is created for view_center with arguments 0.0f and 20.0f. <br> */   
      hz(60.0f),                /** hz is given default value as 60.0f. <br> */
      velocity_iterations(8),   /** Number of velocity_iterations by default is set 8.<br> */
      position_iterations(3),   /** Number of position_iterations by default is set 3.<br> */
      draw_shapes(1),           /** draw_shapes is set to true<br> */
      draw_joints(1),           /** draw_joints is set to true<br> */
      draw_AABBs(0),            /** draw_AABBs axis aligned bounding boxes is set to false<br> */
      draw_pairs(0),            /** draw_pairs is set to false<br> */
      draw_contact_points(0),   /** draw_contact_points is set to false<br> */
      draw_contact_normals(0),  /** draw_contact_normals is set to false<br> */
      draw_contact_forces(0),   /** draw_contact_forces is set to false<br> */
      draw_friction_forces(0),  /** draw_friction_forces is set to false<br> */
      draw_COMs(0),             /** draw_COMs center of masses is set to false<br> */
      draw_stats(0),            /** draw_stats is set to false<br> */
      draw_profile(0),          /** draw_profile is set to false<br> */
      enable_warm_starting(1),  /** enable_warm_starting is set to true<br> */
      enable_continuous(1),     /** enable_continous is set to true<br> */
      enable_sub_stepping(0),   /** enable_sub_stepping is set to false<br> */
      pause(0),                 /** pause is set to false<br> */
      single_step(0)            /** single_step is set to false<br> */
    {}
    
    b2Vec2 view_center;        /**< Defines the world center on screen in b2Vec2 data type */
    float32 hz;                /**< Defines horizontal length on screen */
    int32 velocity_iterations; /**< Defines no. of velocity-iterations */
    int32 position_iterations; /**< Defines no. of positions-iterations */
    int32 draw_shapes;         /**< Defines whether to draw shapes or not, 1 is meant as true and 0 as false */ 
    int32 draw_joints;         /**< Defines whether to draw joints or not, 1 is meant as true and 0 as false */ 
    int32 draw_AABBs;          /**< Defines whether to draw axis-aligned bounding boxes or not, 1 is meant as true and 0 as false */ 
    int32 draw_pairs;          /**< Defines whether to draw pairs or not, 1 is meant as true and 0 as false */ 
    int32 draw_contact_points; /**< Defines whether to show contact points or not, 1 is meant as true and 0 as false */ 
    int32 draw_contact_normals;/**< Defines whether to show contact normals or not, 1 is meant as true and 0 as false */ 
    int32 draw_contact_forces; /**< Defines whether to show contact forces or not, 1 is meant as true and 0 as false */ 
    int32 draw_friction_forces;/**< Defines whether to show friction forces or not, 1 is meant as true and 0 as false */ 
    int32 draw_COMs;           /**< Defines whether to show Center of Masses or not, 1 is meant as true and 0 as false */ 
    int32 draw_stats;          /**< Defines whether to show statistics or not, 1 is meant as true and 0 as false */ 
    int32 draw_profile;        /**< Defines whether to show profile or not, 1 is meant as true and 0 as false */  
    int32 enable_warm_starting;/**< Defines whether to enable warm starting or not, 1 is meant as true and 0 as false */ 
    int32 enable_continuous;   /**< Defines whether to enable continous movement of shapes or not, 1 is meant as true and 0 as false */ 
    int32 enable_sub_stepping; /**< Defines whether to enable sub_stepping or not, 1 is meant as true and 0 as false */ 
    int32 pause;               /**< Defines whether to enable pause or not, 1 is meant as true and 0 as false */ 
    int32 single_step;         /**< Defines whether to enable single step iteration or not, 1 is meant as true and 0 as false */ 
  };
  
  //! Creates the environment for Animation 
  struct sim_t
  {
    const char *name; /**< Name of the world */
    sim_create_fcn *create_fcn; /**< Pointer to the world */

    //! Constructor function
    //! \param _name a character ponter that defines name of the world
    //! \param _create_fcn a pointer to the world
    sim_t(const char *_name, sim_create_fcn *_create_fcn): 
    name(_name), create_fcn(_create_fcn) {;}
  };
  
  extern sim_t *sim;
  
  //! Constant that defines maximum contact points 
  const int32 k_max_contact_points = 2048;  
  
  //! Struct that is used to represent the contact point between two bodies
  struct contact_point_t
  {
    b2Fixture* fixtureA;  //!< Pointer to fixture of body A
    b2Fixture* fixtureB;  //!< Pointer to fixture of body B
    b2Vec2 normal;        //!< Vector representing Normal force acting on the contact point
    b2Vec2 position;      //!< Vector representing Position of Contact Point
    b2PointState state;   //!< Variable to represent the State of Point
  };
  
//! \brief base_sim_t is a class which has functions for controlling simulation and GUI of the Box2D.   
  class base_sim_t : public b2ContactListener
  {
  public:
    
    base_sim_t();

    //! Virtual destructors - amazing objects. Why are these necessary?
    //! Virtual destructors are needed to delete an instance of derived class through a pointer to base class.
    //! Here derived class is base_sim_t and base classe is b2ContactListener 
    virtual ~base_sim_t();
     
    //! Takes and int32 variable as input as text lines in the world as input.
    //! \param line An int32 variable used to set no. of text lines.
    void set_text_line(int32 line) { m_text_line = line; }
     
    //! Creates title on the screen
    //! \param x int variable defines x-coordinate of title
    //! \param y int variable defines y-coordinate of title
    //! \param string pointer to char defines Title to be written
    void draw_title(int x, int y, const char *string);
    
    //! Takes input as pointer to setting and performs next iteration
    //! \param settings Pointer to settings_t struct
    virtual void step(settings_t* settings);

    //! Action to perform when keyboard key is pressed not used here
    //! \param key unsigned char which represents which key is pressed
    virtual void keyboard(unsigned char key) { B2_NOT_USED(key); }
    
    //! Action to perform when keyboard key is pressed not used here
    //! \param key unsigned char which represents which key is pressed
    virtual void keyboard_up(unsigned char key) { B2_NOT_USED(key); }

    //! Action to perform when mouse is clicked not used here
    //! \param p b2Vec2 type which represents at what position on screen the mouse was clicked
    void shift_mouse_down(const b2Vec2& p) { B2_NOT_USED(p); }

    //! Action to perform when mouse is clicked not used here
    //! \param p b2Vec2 type which represents at what position on screen the mouse was clicked
    virtual void mouse_down(const b2Vec2& p) { B2_NOT_USED(p); }

    //! Action to perform when mouse is clicked not used here
    //! \param p b2Vec2 type which represents at what position on screen the mouse was clicked
    virtual void mouse_up(const b2Vec2& p) { B2_NOT_USED(p); }

    //! Action to perform when mouse is clicked not used here
    //! \param p b2Vec2 type which represents at what position on screen the mouse was clicked
    void mouse_move(const b2Vec2& p) { B2_NOT_USED(p); }

    
    // Let derived tests know that a joint was destroyed.
    virtual void joint_destroyed(b2Joint* joint) { B2_NOT_USED(joint); }
    
    // Callbacks for derived classes.
    virtual void begin_contact(b2Contact* contact) { B2_NOT_USED(contact); }
    virtual void end_contact(b2Contact* contact) { B2_NOT_USED(contact); }
    virtual void pre_solve(b2Contact* contact, const b2Manifold* oldManifold);
    virtual void post_solve(const b2Contact* contact, const b2ContactImpulse* impulse)
    {
      B2_NOT_USED(contact);
      B2_NOT_USED(impulse);
    }

  //! How are protected members different from private memebers of a class in C++ ?
  //! Protected classes can also be accessed by friend class and classes derived with public or protected access from the class that 
  //! originally declared these members, in short they are not as private as private class and accessible to classes defined outside.
  protected:

    //! What are Friend classes?
    //! Friend classes are classes that have access to all members including private and protected members of class whose friend it has 
    //! been declared
    friend class contact_listener_t;
    
    b2Body* m_ground_body; /**< Pointer to Box2D body */ 
    b2AABB m_world_AABB;   /**< A axis-aligned bounding box */
    contact_point_t m_points[k_max_contact_points];/**< Creating an array of max contact points declared earlier */
    int32 m_point_count;  /**< keeping a count of points */

    debug_draw_t m_debug_draw; /**< For debugging any errors while drawing */
    int32 m_text_line; /**< No. of text lines */
    b2World* m_world;  /**< Creating a pointer to Box2D world */

    int32 m_step_count;  /**< Keeping a count of steps */
    
    b2Profile m_max_profile; /**< Box2D profile */
    b2Profile m_total_profile; /**< Box2D profile */
  };
}

#endif
