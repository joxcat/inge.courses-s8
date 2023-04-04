package com.example.pokesocks13.database

import com.example.pokesocks13.database.DatabaseFactory.query
import com.example.pokesocks13.models.Pokemon
import com.example.pokesocks13.models.PokemonType
import org.jetbrains.exposed.sql.*

internal object Pokemons : Table() {
    val id = integer("id").autoIncrement()
    val name = varchar("name", 128)
    val type = enumeration("type", PokemonType::class)
    val attack = integer("attack").default(10)
    val defense = integer("defense").default(10)
    val weight = float("weight").default(0.0.toFloat())
    val height = float("height").default(0.0.toFloat())
    val discovered = bool("discovered").default(false)
    val image = integer("image").nullable()

    override val primaryKey = PrimaryKey(id)
}

object PokemonDao {
    private fun rowToPokemon(row: ResultRow) = Pokemon(
        id = row[Pokemons.id],
        name = row[Pokemons.name],
        type = row[Pokemons.type],
        attack = row[Pokemons.attack],
        defense = row[Pokemons.defense],
        weight = row[Pokemons.weight],
        height = row[Pokemons.height],
        discovered = row[Pokemons.discovered],
        image = row[Pokemons.image]
    )

    suspend fun addPokemon(
        name: String,
        type: PokemonType,
        attack: Int? = null,
        defense: Int? = null,
        weight: Float? = null,
        height: Float? = null,
        discovered: Boolean? = false,
        image: Int
    ): Pokemon = query {
        rowToPokemon(Pokemons.insert {
            it[Pokemons.name] = name
            it[Pokemons.type] = type
            if (attack != null) it[Pokemons.attack] = attack
            if (defense != null) it[Pokemons.defense] = defense
            if (weight != null) it[Pokemons.weight] = weight
            if (height != null) it[Pokemons.height] = height
            if (discovered != null) it[Pokemons.discovered] = discovered
            it[Pokemons.image] = image
        }.resultedValues!!.first())
    }

    suspend fun getPokemon(id: Int): Pokemon? = query {
        Pokemons
            .select { Pokemons.id eq id }
            .map(::rowToPokemon)
            .singleOrNull()
    }

    suspend fun getAllPokemons(): List<Pokemon> = query {
        Pokemons.selectAll().map(::rowToPokemon)
    }

    suspend fun encounterPokemon(id: Int): Boolean = query {
        Pokemons.update({ Pokemons.id eq id }) {
            it[discovered] = true
        } > 0
    }

    suspend fun pokemonExist(id: Int): Boolean = query {
        Pokemons.select { Pokemons.id eq id }.count() > 0
    }
}