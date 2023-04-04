package com.example.pokesocks13.database

import android.content.Context
import kotlinx.coroutines.Dispatchers
import org.jetbrains.exposed.sql.Database
import org.jetbrains.exposed.sql.SchemaUtils
import org.jetbrains.exposed.sql.transactions.TransactionManager
import org.jetbrains.exposed.sql.transactions.experimental.newSuspendedTransaction
import org.jetbrains.exposed.sql.transactions.transaction
import java.sql.Connection

object DatabaseFactory {
    fun init(context: Context) {
        val database = Database.connect(
            "jdbc:h2:${context.filesDir.absolutePath}/pokesocks.db",
            driver = "org.h2.Driver"
        )
        TransactionManager.manager.defaultIsolationLevel = Connection.TRANSACTION_SERIALIZABLE

        transaction(database) {
            createTables()
        }
    }

    fun dropTables() {
        SchemaUtils.drop(TeamPokemons)
        SchemaUtils.drop(Pokemons)
    }

    fun createTables() {
        SchemaUtils.create(Pokemons)
        SchemaUtils.create(TeamPokemons)
    }

    suspend fun <T> query(block: suspend () -> T): T =
        newSuspendedTransaction(Dispatchers.IO) { block() }
}