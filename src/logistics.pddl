(define (domain logistics)
   ;editor session: O2kjJjyXvVj1ZjB
   (:requirements :typing :negative-preconditions :conditional-effects)
   (:types
      location locatable slot - object
      box robot person supplies carrier - locatable
      food medicine tools - supplies
   )

   (:predicates

      (in ?s - supplies ?b - box) ; supp in a box

      (at ?obj - locatable ?loc - location) ;obj in location
      (on ?box - box ?car - carrier) ; box is on the carrier

      (empty ?box - box)
      (full ?box - box) ;box vuota

      (assigned ?supplies - supplies) ;risorsa assegnata al cristiano
      (assignable ?supplies - supplies)

      (needs-food ?person - person)
      (needs-medicine ?person - person)
      (needs-tools ?person - person)

      (free ?slot - slot)

      (belongs ?car - carrier ?slot - slot)
   )

   (:action FILL_BOX
      :parameters (?b - box ?loc - location ?robot - robot ?sup - supplies)
      :precondition (and (empty ?b)
         (at ?b ?loc)
         (at ?sup ?loc)
         (at ?robot ?loc)
         (assignable ?sup))
      :effect (and (not (empty ?b))
         (not (at ?sup ?loc))(full ?b)(in ?sup ?b))
   )

   (:action GIVE_TOOLS
      :parameters ( ?b - box ?loc - location ?robot - robot ?tool - tools ?p - person)

      :precondition (and (full ?b)(in ?tool ?b)(needs-tools ?p)
         (at ?b ?loc) (at ?p ?loc) (at ?robot ?loc) (assignable ?tool))

      :effect (and
         (not(full ?b)) (not (in ?tool ?b)) (empty ?b) (assigned ?tool)
         (not(needs-tools ?p))
         (not(assignable ?tool)))
   )

   (:action GIVE_MEDICINE
      :parameters ( ?b - box ?loc - location ?robot - robot ?medicine - medicine ?p - person)

      :precondition (and (full ?b)(in ?medicine ?b)(needs-medicine ?p)
         (at ?b ?loc) (at ?p ?loc) (at ?robot ?loc) (assignable ?medicine))

      :effect (and
         (not(full ?b)) (not (in ?medicine ?b)) (empty ?b) (assigned ?medicine)
         (not(needs-medicine ?p))
         (not(assignable ?medicine)))
   )

   (:action GIVE_FOOD
      :parameters ( ?b - box ?loc - location ?robot - robot ?food - food ?p - person)

      :precondition (and (full ?b)(in ?food ?b)(needs-food ?p)
         (at ?b ?loc) (at ?p ?loc) (at ?robot ?loc) (assignable ?food))

      :effect (and
         (not(full ?b)) (not (in ?food ?b)) (empty ?b) (assigned ?food)
         (not(needs-food ?p))
         (not(assignable ?food)))
   )

   (:action MOVING ;sposta il robot vuoto
      :parameters ( ?start - location ?stop - location ?robot - robot)
      :precondition (at ?robot ?start)
      :effect (and(at ?robot ?stop) (not(at ?robot ?start)))
   )

   (:action MOVING_CARRIER ;sposta il robot e il carrello
      :parameters (?start - location ?stop - location ?robot - robot ?car - carrier)
      :precondition (and (at ?robot ?start) (at ?car ?start))
      :effect (and (not (at ?robot ?start)) (not(at ?car ?start)) (at ?robot ?stop) (at ?car ?stop))
   )

   (:action LOAD_BOX
      :parameters ( ?b - box ?loc - location ?robot - robot ?car - carrier ?slot - slot)
      :precondition (and (at ?b ?loc) (at ?robot ?loc) (at ?car ?loc) (free ?slot) (belongs ?car ?slot))
      :effect (and (on ?b ?car) (not (at ?b ?loc)) (not (free ?slot)))
   )


   (:action UNLOAD_BOX
      :parameters (?b - box ?loc - location ?robot - robot ?car - carrier ?slot - slot)
      :precondition (and (at ?robot ?loc) (at ?car ?loc) (on ?b ?car) (not (free ?slot))(belongs ?car ?slot))
      :effect (and (not (on ?b ?car)) (at ?b ?loc) (free ?slot))
   )
)