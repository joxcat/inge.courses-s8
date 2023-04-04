package com.example.pokesocks13

import android.content.Intent
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.widget.Button
import com.example.pokesocks13.database.PokemonDao
import com.example.pokesocks13.database.TeamDao
import kotlinx.coroutines.runBlocking

class StarterActivity : AppCompatActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        // Hides header
        supportActionBar?.hide()

        // Sets the layout
        setContentView(R.layout.activity_starter)

        val bulbizareButton = findViewById<Button>(R.id.button_bulbizare)
        bulbizareButton.setOnClickListener {
            selectStarter(1)
        }

        val salamecheButton = findViewById<Button>(R.id.button_salameche)
        salamecheButton.setOnClickListener {
            selectStarter(4)
        }

        val carapuceButton = findViewById<Button>(R.id.button_carapuce)
        carapuceButton.setOnClickListener {
            selectStarter(7)
        }
    }

    private fun selectStarter(id: Int) = runBlocking {
        TeamDao.addTeamPokemon(id, 1)
        PokemonDao.encounterPokemon(id)

        // Leaves the activity
        val intent = Intent(this@StarterActivity, MainActivity::class.java)
        startActivity(intent)
    }
}