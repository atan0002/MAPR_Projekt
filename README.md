 # Projekt: Implementacja modułu do planowania ruchu manipulatora UR5 w ROS 2 (Humble) z wykorzystaniem OMPL  

 ## Przedmiot : Metody i algorytmy planowania ruchu

### W projekcie zostały wykorzystane narzędzia:

- ROS2 Humble
- MoveIt
- Biblioteka OMPL
- RViz
- Docker
- Symulator robota ur5


### Opis projektu

 Projekt miał na celu zaimplementowanie planera z natywnym wykorzystyniem biblioteki OMPL oraz modułów MoveIt. Pozycja zadawana jest w przestrzeni przegubów.


Pozycja planowana jest poprzez funkcję plan(). Na początku inicjowana jest klasa definicji problemu, do której przypisywany jest punkt startowy i końcowy. Następnie definiowany jest planer. W tym przypadku wybrany został RRTConnect. Przypisywana jest do niego informacja o przestrzeni stanów. Następnie następuje zaplanowanie ścieżki oraz jej wyświetlenie.

```cpp
auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    
pdef->setStartAndGoalStates(start, goal);

auto planner(std::make_shared<og::RRTConnect>(si));
planner->setProblemDefinition(pdef);

planner->setup();

ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);
```
Informacje wyśiwetlane przez funckję planującą.

```
[plan_trajectory-1] Info:    RRTConnect: Space information setup was not yet called. Calling now.
[plan_trajectory-1] Debug:   RRTConnect: Planner range detected to be 1.539060
[plan_trajectory-1] Info:    RRTConnect: Starting planning with 1 states already in datastructure
[plan_trajectory-1] Info:    RRTConnect: Created 5 states (2 start + 3 goal)
[plan_trajectory-1] Found solution:
[plan_trajectory-1] Geometric path with 4 states
[plan_trajectory-1] RealVectorState [0 -1.57 0 -1.57 0 0]
[plan_trajectory-1] RealVectorState [0.165963 -0.720384 -0.205904 -0.351046 0.0286827 0.300409]
[plan_trajectory-1] RealVectorState [0.15182 -0.658997 -0.121466 -0.321132 0.0262385 0.207917]
[plan_trajectory-1] RealVectorState [0 0 0.785 0 0 -0.785]
[plan_trajectory-1] 

```

Następnie wykonywana jest wizualizacja. Wyznaczona ścieżka przesyłana jest na temat /display_planned_path.

### Planowanie i wizualizacja za pomocą MoveIt
Planowanie i wizualizacja odbywa się za pomocą biblioteki MoveIt w przestrzeni przegubów. Rozwiązanie to jest identyczne do wyżej opisanego, gdzie zadajemy punkt końcowy definiując wartości w przegubach. Następnie ścieżka jest planowana oraz wyświetlana w RViz.

```cpp
move_group_interface.setJointValueTarget(target_joint_vals);
    auto const [success, plan] = [&move_group_interface]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    if (success)
    {
        draw_trajectory_tool_path(plan.trajectory_);
        std::cout<<"Executing!"<<std::endl;
        move_group_interface.execute(plan);
    }
    else
        std::cout<<"Planning failed!"<<std::endl;
```
Informacje wyświetlane przez funkcję planującą z MoveIt:
```
[plan_trajectory_with_viz-1] [INFO]: ########## Joint states - start ##########
[plan_trajectory_with_viz-1] [INFO]: Joint shoulder_pan_joint: 0.000000
[plan_trajectory_with_viz-1] [INFO]: Joint shoulder_lift_joint: 0.000000
[plan_trajectory_with_viz-1] [INFO]: Joint elbow_joint: 0.000000
[plan_trajectory_with_viz-1] [INFO]: Joint wrist_1_joint: 0.000000
[plan_trajectory_with_viz-1] [INFO]: Joint wrist_2_joint: 0.000000
[plan_trajectory_with_viz-1] [INFO]: Joint wrist_3_joint: 0.000000
[plan_trajectory_with_viz-1] [INFO]: Ready to take commands for planning group ur_manipulator.
[plan_trajectory_with_viz-1] [INFO]: RemoteControl Ready.
[plan_trajectory_with_viz-1] [INFO]: MoveGroup action client/server ready
[plan_trajectory_with_viz-1] [INFO]: Planning request accepted
[plan_trajectory_with_viz-1] [INFO]: Planning request complete!
[plan_trajectory_with_viz-1] [INFO]: time taken to generate plan: 0.0241243 seconds
[plan_trajectory_with_viz-1] Executing!
[plan_trajectory_with_viz-1] [INFO]: Execute request accepted
[plan_trajectory_with_viz-1] [INFO]: Execute request success!
[plan_trajectory_with_viz-1] [INFO]: ########## Joint states - goal ##########
[plan_trajectory_with_viz-1] [INFO]: Joint shoulder_pan_joint: -0.000023
[plan_trajectory_with_viz-1] [INFO]: Joint shoulder_lift_joint: 0.784949
[plan_trajectory_with_viz-1] [INFO]: Joint elbow_joint: 0.000015
[plan_trajectory_with_viz-1] [INFO]: Joint wrist_1_joint: 0.000025
[plan_trajectory_with_viz-1] [INFO]: Joint wrist_2_joint: -0.000098
[plan_trajectory_with_viz-1] [INFO]: Joint wrist_3_joint: -0.784949
```


### Napotkane problemy

- Konflikty z podlinkowaniem biblioteki OMPL i MoveIt,
- Nie znaleziono możliwości wykorzystania metody execute() z move_group_interface,
- niezaktualizowana dokumentacja od biblioteki OMPL oraz MoveIt 
