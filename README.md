# TakeOff2
Paczka tworzy behavior używany przez Aerostack (oprogramowanie grupy Vision4UAV: https://github.com/Vision4UAV/Aerostack)
Behavior TakeOff2 powoduje, że UAV startuje i zawisa na wysokości podanej przez użytkownika (co najmniej 0.7m)
### Instalacja ###
1. Pliki niniejszego repozytorium należy umieścić w folderze 
    `~/workspace/ros/aerostack_catkin_ws/src/`
    tak, aby tworzyły poniższe drzewo:
    
        ~/workspace/ros/aerostack_catkin_ws/src/
            -behavior_take_off2
                -CMakeLists.txt
                -package.xml
                -launch
                    -behavior_take_off2.launch
    			-src
                    -include
                        -behavior_take_off2.h
                    -source
                        -behavior_take_off2.cpp
                        -behavior_take_off2_main.cpp

2. Przeprowadzić kompilację catkin `~/workspace/ros/aerostack_catkin_ws/$ catkin_make`
3. Edytować plik `simulated_quadrotor_basic.sh` - W skrypcie uruchamiającym należy dokleić na końcu poniższe linie:
    
	    `#----------------------------------------------` \
	    `# Behavior TakeOff2                                    ` \
	    `#----------------------------------------------` \
	    --tab --title "Behavior TakeOff2" --command "bash -c \"
	    roslaunch behavior_take_off2 behavior_take_off2.launch --wait \
    		drone_id_namespace:=drone$NUMID_DRONE \
    		drone_id:=$NUMID_DRONE \
    		my_stack_directory:=${AEROSTACK_STACK};
    	exec bash\"" \
    
4. Edytować plik `behavior_catalog.yaml`. Plik znajduje się w lokalizacji: `~/workspace/ros/aerostack_catkin_ws/src/aerostack_stack/configs/drone1` 
    W sekcji `behavior_descriptors` należy dokleić poniższe linie:
	    
		- behavior: TAKE_OFF2
				timeout: 30
				incompatible_lists: [motion_behaviors]
				arguments:
				  - argument: ALTITUDE
					allowed_values: [0.7,5]
##### UWAGA! Wcięcia powinny być realizowane przez spacje, nie tabulatory!

### Przyjmowane argumenty ###
Behavior przyjmuje argumenty:
    
    altitude = x` //wysokość w metrach na jaką dron ma się wznieść. 0.7<x<5
    
Przykład wywołania:
`result = api.executeBehavior('TAKE_OFF2', altitude=1.2)`