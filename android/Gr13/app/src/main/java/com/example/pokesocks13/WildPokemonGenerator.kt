package com.example.pokesocks13

import androidx.databinding.BaseObservable
import androidx.databinding.Bindable
import androidx.databinding.ObservableArrayList
import com.example.pokesocks13.database.PokemonDao
import com.example.pokesocks13.database.TeamDao
import com.example.pokesocks13.fragments.TeamFragment
import com.example.pokesocks13.models.Pokemon
import com.example.pokesocks13.models.Position
import com.example.pokesocks13.models.WildPokemon
import kotlinx.coroutines.runBlocking
import java.time.Instant
import java.util.*
import kotlin.concurrent.scheduleAtFixedRate
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.random.Random

class WildPokemonGenerator(
    private var pokemons: List<Pokemon>,
    private var userLocation: Position,
    // TODO: Setup another way to update the fragment's list with signals
    private var teamFragment: TeamFragment,
    everyMs: Long = 10000,
    maxAmount: Int = 20,
    maxLifetime: Long = 60 * 10
) : BaseObservable() {

    private val MIN_METERS_OFFSET: Double = 30.0
    private val MAX_METERS_OFFSET: Double = 500.0
    private val DESPAWN_METERS_OFFSET: Double = 5000.0
    private val RADIUS: Double = MAX_METERS_OFFSET - MIN_METERS_OFFSET
    private val CATCH_RADIUS_METERS: Double = 5.0
    private val ENCOUNTER_RADIUS_METERS: Double = 20.0
    
    @get:Bindable
    var wildPokemons: ObservableArrayList<WildPokemon> = ObservableArrayList()
    private var lastGeneration = Instant.now()

    val task = Timer().scheduleAtFixedRate(0, 250) {
        var wildPokemonsHaveChanged = false

        // Cleaning pokemons
        val iterator = wildPokemons.iterator()
        while (iterator.hasNext()) {
            val pokemon = iterator.next()
            val playerDistance = pokemon.position.distanceInMeters(userLocation)

            // TODO : remove after discovering pokemon within fight
            if (playerDistance > DESPAWN_METERS_OFFSET) {
                iterator.remove()
                wildPokemonsHaveChanged = true
            }
            else if (playerDistance <= CATCH_RADIUS_METERS) {
                runBlocking {
                    // Encounters the pokemon
                    runBlocking {
                        PokemonDao.encounterPokemon(pokemon.id!!)
                    }
                    pokemons.find { p -> p.id == pokemon.id!! }?.let {
                        it.discovered = true;
                    }

                    // If the pokemon isn't in the team yet, add it
                    if (!TeamDao.pokemonExistsInTeam(pokemon.id!!)) {
                        val nextAvailablePosition = TeamDao.nextAvailablePosition()
                        TeamDao.addTeamPokemon(pokemon.id!!, nextAvailablePosition)
                        teamFragment.setPokemonList(TeamDao.getWholeTeam())
                    }
                }

                iterator.remove()
                wildPokemonsHaveChanged = true
            }
            else if (playerDistance <= ENCOUNTER_RADIUS_METERS) {
                runBlocking {
                    PokemonDao.encounterPokemon(pokemon.id!!)
                }
                pokemons.find { p -> p.id == pokemon.id!! }?.let {
                    it.discovered = true;
                }

                wildPokemonsHaveChanged = true
            }
            else if (Instant.now().epochSecond - pokemon.spawnedTime.epochSecond > maxLifetime) {
                // Pokemon is despawning
                iterator.remove()
                wildPokemonsHaveChanged = true
            }
        }

        // Generating pokemons
        val now = Instant.now()
        if (now.toEpochMilli() - lastGeneration.toEpochMilli() > everyMs) {
            val pokemonCountInRadius = wildPokemons.fold(0) { count, poke ->
                if (poke.position.distanceInMeters(userLocation) <= MAX_METERS_OFFSET) count + 1
                else count
            }

            if (pokemonCountInRadius < maxAmount) {
                val pokemon = pokemons[Random.nextInt(0, pokemons.size)]

                // Source: https://stackoverflow.com/questions/5837572/generate-a-random-point-within-a-circle-uniformly
                val r = RADIUS * sqrt(Random.nextDouble(0.0, 1.0)) + MIN_METERS_OFFSET
                val theta = Random.nextDouble(0.0, 1.0) * 2 * PI
                val offsetX = r * cos(theta)
                val offsetY = r * sin(theta)

                val pokemonPos = Position(userLocation).offsetInMeters(offsetX, offsetY)

                wildPokemons.add(WildPokemon(pokemon, pokemonPos))

                wildPokemonsHaveChanged = true
            }

            lastGeneration = now
        }

        if (wildPokemonsHaveChanged) notifyPropertyChanged(BR.wildPokemons)
    }
}