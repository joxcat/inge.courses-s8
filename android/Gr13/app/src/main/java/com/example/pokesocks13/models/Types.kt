package com.example.pokesocks13.models

import kotlinx.serialization.Serializable

@Serializable
enum class PokemonType {
    Steel,
    Fight,
    Dragon,
    Water,
    Electric,
    Fairy,
    Fire,
    Ice,
    Insect,
    Normal,
    Plant,
    Poison,
    Psy,
    Rock,
    Ground,
    Spectre,
    Darkness,
    Flight,
    Sock
}

fun frenchStringToType(type: String): PokemonType {
    return when (type) {
        "Acier" -> PokemonType.Steel
        "Combat" -> PokemonType.Fight
        "Dragon" -> PokemonType.Dragon
        "Eau" -> PokemonType.Water
        "Electrique" -> PokemonType.Electric
        "Fee" -> PokemonType.Fairy
        "Feu" -> PokemonType.Fire
        "Glace" -> PokemonType.Ice
        "Insecte" -> PokemonType.Insect
        "Normal" -> PokemonType.Normal
        "Plante" -> PokemonType.Plant
        "Poison" -> PokemonType.Poison
        "Psy" -> PokemonType.Psy
        "Roche" -> PokemonType.Rock
        "Sol" -> PokemonType.Ground
        "Spectre" -> PokemonType.Spectre
        "Tenebres" -> PokemonType.Darkness
        "Vol" -> PokemonType.Flight
        "Chaussette" -> PokemonType.Sock
        else -> error("Unexpected type: $type")
    }
}

fun typeToFrenchString(type: PokemonType): String {
    return when (type) {
        PokemonType.Steel -> "Acier"
        PokemonType.Fight -> "Combat"
        PokemonType.Dragon -> "Dragon"
        PokemonType.Water -> "Eau"
        PokemonType.Electric -> "Electrique"
        PokemonType.Fairy -> "Fee"
        PokemonType.Fire -> "Feu"
        PokemonType.Ice -> "Glace"
        PokemonType.Insect -> "Insecte"
        PokemonType.Normal -> "Normal"
        PokemonType.Plant -> "Plante"
        PokemonType.Poison -> "Poison"
        PokemonType.Psy -> "Psy"
        PokemonType.Rock -> "Roche"
        PokemonType.Ground -> "Sol"
        PokemonType.Spectre -> "Spectre"
        PokemonType.Darkness -> "Tenebres"
        PokemonType.Flight -> "Vol"
        PokemonType.Sock -> "Chaussette"
    }
}