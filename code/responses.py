"""
This file contains funny error messages to display
when users input values outside the expected range.

@Author: Jaysh Khan
@response author: Gemini
"""
import random


def get_error_response(min_value, max_value):
    funny_responses = [
        "Whoa there, cowboy! That number is wilder than my weekend plans. Enter a value between [min value] and [max value].",
        "Is that a typo? It looks like your fingers got a little too excited on the keyboard. Pick a number from [min value] to [max value] this time.",
        "Hold on a sec, are you sure you didn't accidentally type in your phone number? We need a value between [min value] and [max value] here.",
        "Nope, that number isn't on the menu today. Try again with a value between [min value] and [max value].",
        "Did you just try to break the system? We only accept numbers between [min value] and [max value]. Don't be a rebel!",
        "That number sounds like it belongs in a galaxy far, far away. We need something a little closer to home - between [min value] and [max value].",
        "Are you using a special measuring system? Ours uses values from [min value] to [max value]. Let's stick to that, shall we?",
        "Maybe you should consult a fortune teller for that number. We only deal in numbers between [min value] and [max value] here.",
        "Looks like you forgot to take your daily dose of common sense. Fear not, a normal number (between [min value] and [max value]) will do just fine.",
        "That number is so out of range, it needs its own passport! Pick a number that can stay within [min value] and [max value].",
        "Whoa, slow down there Speed Racer! We're cruising in the range between [min value] and [max value], not racing off into the sunset.",
        "Did you mistake our input field for a lottery ticket? Keep it grounded within [min value] and [max value] for this ride.",
        "I'm all for exploring new frontiers, but let's keep our exploration within the confines of [min value] and [max value], shall we?",
        "Hmm, I think you're confusing our system with a high-stakes poker game. Stick to the safe bets between [min value] and [max value].",
        "Looks like you're trying to give our system a heart attack with that number. Let's stick to numbers that won't cause a system meltdown, okay?",
        "I'm afraid that number just took a quantum leap out of our range. Let's bring it back down to Earth, between [min value] and [max value].",
        "Is this a math problem or a riddle? Either way, the solution lies between [min value] and [max value].",
        "If that number were a movie, it'd be rated 'NC-Not in the Calculator'. Stick to numbers within [min value] and [max value] for this feature film.",
        "You're reaching for the stars with that number! Let's keep our feet on the ground and within the range of [min value] and [max value].",
        "I think you just discovered the secret code to unlock the mysteries of the universe! But for this system, let's stick to the tried and true between [min value] and [max value]."
    ]

    return random.choice(funny_responses).replace("[min value]", str(min_value)).replace("[max value]", str(max_value))


def get_error_rresponses_for_singularity():
    funny_responses = [
        "Oh no! You've hit a singularity! This is like dividing by zero. Try a different set of angles.",
        "Uh oh! Looks like your angle array is suffering from a serious case of 'too close for comfort.' Reshaping the design to avoid a singularity... forever.",
        "Yikes! Those angles in your array are more dangerous than a black hole convention. Going to have to put a pin in this one to avoid a self-collision catastrophe.",
        "Hold on there! Building a time machine with your angle array, are we? This path leads to a singularity, not someplace cool. Better adjust those angles before you break the space-time continuum.",
        "You sure those angles in your array aren't a typo? Because the only thing they're designing is a recipe for disaster. Let's revisit those angles and avoid a self-collision incident.",
        "Houston, we have a problem... with your angle array! They're creating a situation more tense than a taut rubber band. Back to the drawing board to avoid a self-inflicted ouch.",
    ]
    return random.choice(funny_responses)
