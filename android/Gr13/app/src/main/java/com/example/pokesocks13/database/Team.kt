package com.example.pokesocks13.database

import com.example.pokesocks13.database.DatabaseFactory.query
import com.example.pokesocks13.models.TeamPokemon
import org.jetbrains.exposed.sql.*

internal object TeamPokemons : Table() {
    val id = integer("id").autoIncrement()
    val pokemon_id = integer("pokemon_id").references(Pokemons.id)
    val position = integer("position")

    override val primaryKey = PrimaryKey(id)
}

object TeamDao {
    private fun rowToTeamPokemon(row: ResultRow) = TeamPokemon(
        id = row[TeamPokemons.id],
        pokemonId = row[TeamPokemons.pokemon_id],
        position = row[TeamPokemons.position],
    )

    suspend fun addTeamPokemon(
        pokemonId: Int,
        position: Int,
    ): TeamPokemon = query {
        rowToTeamPokemon(TeamPokemons.insert {
            it[TeamPokemons.pokemon_id] = pokemonId
            it[TeamPokemons.position] = position
        }.resultedValues!!.first())
    }

    suspend fun getTeamPokemon(id: Int): TeamPokemon? = query {
        TeamPokemons
            .select { TeamPokemons.id eq id }
            .map(::rowToTeamPokemon)
            .singleOrNull()
    }

    suspend fun getWholeTeam(): List<TeamPokemon> = query {
        TeamPokemons.selectAll().map(::rowToTeamPokemon)
    }

    suspend fun pokemonExistsInTeam(id: Int): Boolean = query {
        TeamPokemons.select { TeamPokemons.pokemon_id eq id }.count() > 0
    }

    suspend fun nextAvailablePosition(): Int = query {
        TeamPokemons
            .slice(TeamPokemons.position.max()).selectAll()
            .maxBy { TeamPokemons.position }[TeamPokemons.position.max()].let {
                return@query it!! + 1
        }
    }
}