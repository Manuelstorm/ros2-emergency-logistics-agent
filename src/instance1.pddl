(define (problem instance1)
    (:domain logistics)
    (:requirements :negative-preconditions :universal-preconditions)
    (:objects
        box1 box2 box3 box4 box5 - box
        depot pos1 pos2 pos3 - location
        person1 person2 person3 - person
        banana1 banana2 banana3 - food
        aspirine1 aspirine2 aspirine3 - medicine
        s1 s2 s3 s4 - slot
        robot1 - robot
        carrier1 - carrier
    )

    ;definire un numero fissato di oggetti counter, per poi richiamare forall sugli oggetti e contare tramite essi

    (:init
        (at box1 depot)
        (at box2 depot)
        (at box3 depot)
        (at box4 depot)
        (at box5 depot)
        (empty box1)(empty box2)(empty box3)(empty box4)(empty box5)

        (at robot1 pos3)
        (at carrier1 pos3)
        (free s1)(free s2)(free s3)(free s4)
        (belongs carrier1 s1)(belongs carrier1 s2)(belongs carrier1 s3)(belongs carrier1 s4)

        (at banana1 depot)
        (at banana2 depot)
        (at banana3 depot)
        (at aspirine1 depot)
        (at aspirine2 depot)
        (at aspirine3 depot)
        (assignable aspirine1)
        (assignable aspirine2)
        (assignable aspirine3)
        (assignable banana1)
        (assignable banana2)
        (assignable banana3)

        (at person1 pos1)
        (at person2 pos1)
        (at person3 pos2)

        (needs-food person1)
        (needs-medicine person1)
        (needs-medicine person2)
        (needs-food person3)

    )

    (:goal
        (forall
            (?p - person)
            (and
                (not (needs-food ?p))
                (not (needs-medicine ?p))
                (not (needs-tools ?p))
            )
        )
    )

)