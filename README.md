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


### Napotkane problemy

- Konflikty z podlinkowaniem biblioteki OMPL i MoveIt,
- Nie znaleziono możliwości wykorzystania metody execute() z move_group_interface,
- niezaktualizowana dokumentacja od biblioteki OMPL oraz MoveIt 
