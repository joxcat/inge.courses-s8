package com.example.pokesocks13

import android.Manifest
import android.app.AlertDialog
import android.content.Intent
import android.content.pm.PackageManager
import android.os.Bundle
import android.os.Handler
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.databinding.DataBindingUtil
import androidx.fragment.app.Fragment
import androidx.fragment.app.FragmentManager
import com.example.pokesocks13.database.DatabaseFactory
import com.example.pokesocks13.database.PokemonDao
import com.example.pokesocks13.database.TeamDao
import com.example.pokesocks13.databinding.ActivityMainBinding
import com.example.pokesocks13.fragments.MapFragment
import com.example.pokesocks13.fragments.PokedexFragment
import com.example.pokesocks13.fragments.PokedexPageFragment
import com.example.pokesocks13.fragments.TeamFragment
import com.example.pokesocks13.listeners.OnClickOnPokemonItemListener
import com.example.pokesocks13.models.Pokemon
import com.example.pokesocks13.models.Position
import com.example.pokesocks13.models.TeamPokemon
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import kotlinx.serialization.decodeFromString
import kotlinx.serialization.json.Json

class MainActivity : AppCompatActivity() {

    private lateinit var binding: ActivityMainBinding
    private lateinit var manager: FragmentManager
    private var mapFragment: MapFragment = MapFragment()
    private var pokedexFragment: PokedexFragment = PokedexFragment()
    private var teamFragment: TeamFragment = TeamFragment()
    private var pokedexPageFragment: PokedexPageFragment = PokedexPageFragment()
    private lateinit var pokemonList: List<Pokemon>
    private lateinit var team: List<TeamPokemon>
    private lateinit var wildPokemonGenerator: WildPokemonGenerator
    private var userPosition: Position = Position()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        // Hides header
        supportActionBar?.hide()

        binding = DataBindingUtil.setContentView(this, R.layout.activity_main)
        manager = supportFragmentManager

        val listener = object : OnClickOnPokemonItemListener {
            override fun onClickOnPokemonItem(pokemonId: Int) {
                runBlocking {
                    val pokemon = PokemonDao.getPokemon(pokemonId)
                    pokemon?.let {
                        pokedexPageFragment.setPokemon(it)
                        setFragment(pokedexPageFragment)
                    }
                }
            }
        }

        // Initialise database
        DatabaseFactory.init(this)

        /*
        // Reset DB
        transaction {
            DatabaseFactory.dropTables()
        }*/

        runBlocking {
            // Imports the pokemon from JSON file if they are not in the DB
            if (PokemonDao.getAllPokemons().isEmpty()) {
                applicationContext.assets.open("poke.json").bufferedReader().use {
                    val jsonString = it.readText()
                    val list = Json.decodeFromString<List<Pokemon>>(jsonString)

                    for (pokemon in list) {
                        pokemon.image = resources.getIdentifier("p" + pokemon.image, "drawable", packageName)
                        PokemonDao.addPokemon(
                            pokemon.name,
                            pokemon.type,
                            pokemon.attack,
                            pokemon.defense,
                            pokemon.weight,
                            pokemon.height,
                            pokemon.discovered,
                            pokemon.image!!
                        )
                    }
                }
            }
        }

        runBlocking {
            // If the team is empty, lets you choose your starter
            if (TeamDao.getWholeTeam().isEmpty()) {
                val intent = Intent(this@MainActivity, StarterActivity::class.java)
                startActivity(intent)
            }
        }

        // Gets pokemon list and sets it to fragment
        runBlocking {
            // Get all the pokemons
            pokemonList = PokemonDao.getAllPokemons()
            // Sets the pokemon list and the click listener for the pokedex fragment
            pokedexFragment.setPokemonList(pokemonList)
            pokedexFragment.setOnClickOnPokemonItemListener(listener)

            // Sets the team and the click listener for the team fragment
            team = TeamDao.getWholeTeam()
            teamFragment.setPokemonList(team)
            teamFragment.setOnClickOnPokemonItemListener(listener)
        }

        // Creates the wild pokemon generator and sets the user's position
        wildPokemonGenerator = WildPokemonGenerator(pokemonList, userPosition, teamFragment)
        mapFragment.setUserPosition(userPosition)
        mapFragment.setWildPokemons(wildPokemonGenerator.wildPokemons)

        // Request permissions
        if (ActivityCompat.checkSelfPermission(
                this,
                Manifest.permission.ACCESS_FINE_LOCATION
            ) != PackageManager.PERMISSION_GRANTED
        ) {
            val permissions = arrayOf(Manifest.permission.ACCESS_FINE_LOCATION)
            ActivityCompat.requestPermissions(this, permissions, 1)
        } else {
            GlobalScope.launch {
                wildPokemonGenerator.task.run()
            }
            setFragment(mapFragment, true)
        }

        binding.navigationMenu.setOnItemSelectedListener {
            if (it.itemId != R.id.action_map) {
                // wildPokemonGenerator.task.cancel()
            }

            when (it.itemId) {
                R.id.action_map -> {
                    GlobalScope.launch {
                        wildPokemonGenerator.task.run()
                    }
                    mapFragment.setUserPosition(userPosition)
                    mapFragment.setWildPokemons(wildPokemonGenerator.wildPokemons)
                    setFragment(mapFragment)
                }
                R.id.action_pokedex -> setFragment(pokedexFragment)
                R.id.action_team -> setFragment(teamFragment)
            }
            true
        }
    }
    
    private fun setFragment(fragment: Fragment, first: Boolean = false) =
        manager.beginTransaction().apply {
            replace(R.id.fragment_container, fragment)
            if (!first) {
                addToBackStack(null)
            }
            commit()
        }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<String?>,
        grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(
            requestCode,
            permissions, grantResults
        )

        if (grantResults[0] == PackageManager.PERMISSION_GRANTED) {
            // we have the perms
            GlobalScope.launch {
                wildPokemonGenerator.task.run()
            }
            setFragment(mapFragment, true)
        } else {
            // we don't
            val alert = AlertDialog.Builder(this)
            alert.setTitle("Permissions")
            alert.setMessage("Toutes les permissions sont requises")
            alert.setPositiveButton("Rééssayer") { _, _ ->
                onRequestPermissionsResult(
                    requestCode,
                    permissions,
                    grantResults
                )
            }
            alert.setNegativeButton("Quitter") { _, _ ->
                finishAndRemoveTask()
            }
            alert.show()
        }
    }
}