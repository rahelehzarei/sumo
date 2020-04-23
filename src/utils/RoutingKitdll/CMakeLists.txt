set(RoutingKitdll_STAT_SRCS
   Customizable_Contraction_Hierarchy.h
   Customizable_Contraction_Hierarchy.cpp
   nested_dissection.cpp
   nested_dissection.h
   constants.h
   bit_vector.h
   bit_vector.cpp
   bit_select.cpp
   bit_select.h
   emulate_gcc_builtin.h
   timer.h
   timer.cpp
   filter.h
   id_set_queue.h
   sort.h
   graph_util.h
   graph_util.cpp
   inverse_vector.h
   permutation.h
   id_mapper.h
   id_mapper.cpp
   min_max.h)
   
add_library(RoutingKitdll STATIC ${RoutingKitdll_STAT_SRCS})
set_property(TARGET RoutingKitdll PROPERTY PROJECT_LABEL "z_routingkitdll")
